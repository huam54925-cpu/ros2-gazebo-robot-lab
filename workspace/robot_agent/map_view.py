"""Online occupancy-map view and metric coordinates; never reads Gazebo truth."""
import base64
import math
import struct
import zlib
import numpy as np


def png(rgb):
    h, w, _ = rgb.shape
    def chunk(kind, data):
        return struct.pack('>I', len(data))+kind+data+struct.pack('>I', zlib.crc32(kind+data)&0xffffffff)
    raw = b''.join(b'\0'+row.tobytes() for row in rgb)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('>IIBBBBB', w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')


def render(grid, pose, epoch, version, traces, contract_hash):
    from map_components import components
    data = grid['data']; h,w=data.shape; r=grid['resolution']; ox,oy,angle=grid['origin']
    scale=max(1,math.ceil(max(h,w)/800))
    rgb=np.full((h,w,3),145,dtype=np.uint8)
    rgb[(data>=0)&(data<25)]=245;rgb[data>=65]=25;rgb[(data>=25)&(data<65)]=[190,160,90]
    def world(ix,iy):
        x,y=(ix+.5)*r,(iy+.5)*r
        return [ox+math.cos(angle)*x-math.sin(angle)*y,oy+math.sin(angle)*x+math.cos(angle)*y]
    def pixel(p):
        x,y=p[0]-ox,p[1]-oy
        return round((math.cos(angle)*x+math.sin(angle)*y)/r-.5),round((-math.sin(angle)*x+math.cos(angle)*y)/r-.5)
    def mark(p,color,size):
        x,y=pixel(p)
        rgb[max(0,y-size):min(h,y+size+1),max(0,x-size):min(w,x+size+1)]=color
    for p in traces:
        mark(p,[40,100,220],max(1,scale))
    free=(data>=0)&(data<25); adjacent=np.zeros_like(free)
    adjacent[1:] |= free[:-1];adjacent[:-1] |= free[1:]
    adjacent[:,1:] |= free[:,:-1];adjacent[:,:-1] |= free[:,1:]
    boundary=(data<0)&adjacent
    regions=[]
    groups=sorted(components(boundary),key=len,reverse=True)
    for i,g in enumerate(groups[:32]):
        iy,ix=np.mean(g,axis=0); center=world(ix,iy)
        region={'region_id':'U'+str(i+1),'map_epoch':epoch,'map_version':version,
                'center_xy':center,'boundary_cells':len(g),
                'image_pixel_xy':[ix/scale,(h-1-iy)/scale],
                'classification':'unknown_boundary_not_room','identity_scope':'this_snapshot'}
        regions.append(region);mark(center,[20,190,110],2*scale)
    mark(pose,[220,45,45],3*scale)
    head=[pose[0]+.7*math.cos(pose[2]),pose[1]+.7*math.sin(pose[2])]
    for f in np.linspace(0,1,20):mark([pose[0]+f*(head[0]-pose[0]),pose[1]+f*(head[1]-pose[1])],[220,45,45],scale)
    rgb=rgb[::-1,::scale][::scale].copy()
    glyphs={'U':('101','101','101','101','111'),'0':('111','101','101','101','111'),
            '1':('010','110','010','010','111'),'2':('111','001','111','100','111'),
            '3':('111','001','111','001','111'),'4':('101','101','111','001','001'),
            '5':('111','100','111','001','111'),'6':('111','100','111','101','111'),
            '7':('111','001','010','010','010'),'8':('111','101','111','101','111'),
            '9':('111','101','111','001','111')}
    for region in regions:
        px,py=(int(round(v)) for v in region['image_pixel_xy'])
        px=max(0,min(rgb.shape[1]-len(region['region_id'])*8,px+4))
        py=max(0,min(rgb.shape[0]-10,py-5))
        for index,ch in enumerate(region['region_id']):
            for y,row in enumerate(glyphs[ch]):
                for x,on in enumerate(row):
                    if on=='1':rgb[py+2*y:py+2*y+2,px+index*8+2*x:px+index*8+2*x+2]=[0,100,30]
    c,s=math.cos(angle),math.sin(angle)
    affine=[[r*scale*c,r*scale*s,ox+c*r*.5-s*r*(h-.5)],
            [r*scale*s,-r*scale*c,oy+s*r*.5+c*r*(h-.5)]]
    return {'frame_id':grid['frame'],'map_epoch':epoch,'map_version':version,'contract_hash':contract_hash,
            'stamp_sim_s':grid['stamp_sim_s'],'resolution_m':r,'origin_xy_yaw':list(grid['origin']),
            'grid_width':w,'grid_height':h,'robot_pose':pose,'unknown_regions':regions,
            'unknown_regions_total':len(groups),'unknown_regions_truncated':len(groups)>32,
            'trajectory':traces[-200:],'image':{'mime_type':'image/png','base64':base64.b64encode(png(rgb)).decode(),
                'width':int(rgb.shape[1]),'height':int(rgb.shape[0]),'cell_stride':scale,
                'pixel_index_to_world_affine':affine,
                'coordinates':'integer pixel indices (u,v,1) multiplied by the 2x3 affine produce map XY; display sampling is not collision evidence',
                'legend':{'white':'observed free','black':'occupied','gray':'unknown','ochre':'uncertain occupancy',
                          'red':'robot and heading','blue':'trajectory','green':'unknown boundary centers; IDs in metadata'}},
            'world_complete':False,'coverage':None,'source':'online_slam_only'}
