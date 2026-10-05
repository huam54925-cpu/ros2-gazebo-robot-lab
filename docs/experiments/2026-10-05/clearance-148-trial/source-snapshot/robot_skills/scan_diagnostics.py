"""Read-only scan statistics. No interpretation as free/occupied map writes."""
import math


def summarize_scan(ranges,range_min,range_max):
    bounds=math.isfinite(range_min) and math.isfinite(range_max) and 0<=range_min<range_max
    counts=dict(finite_in_range=0,finite_out_of_range=0,positive_inf=0,negative_inf=0,nan=0,at_max_range=0)
    for value in ranges:
        if math.isnan(value):counts['nan']+=1
        elif math.isinf(value):counts['positive_inf' if value>0 else 'negative_inf']+=1
        elif bounds and range_min<=value<=range_max:
            counts['finite_in_range']+=1
            if math.isclose(value,range_max,abs_tol=1e-5,rel_tol=0):counts['at_max_range']+=1
        else:counts['finite_out_of_range']+=1
    return {'counts':counts,'range_bounds_valid':bounds,
            'range_min_m':range_min if math.isfinite(range_min) else None,
            'range_max_m':range_max if math.isfinite(range_max) else None,
            'inf_clearing_semantics':'not_inferred','map_cells_written':0}
