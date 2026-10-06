"""Credential-free error summaries."""
import openai

def safe_error(error):
    if isinstance(error, BaseExceptionGroup):
        return safe_error(error.exceptions[0])
    if isinstance(error, openai.APIStatusError):
        return {'status': 'api_error', 'http_status': error.status_code}
    if isinstance(error, (openai.APIConnectionError, openai.APITimeoutError)):
        return {'status': 'api_connection_error'}
    allowed = {'missing_api_key_or_model', 'unexpected_mcp_tools', 'tool_not_allowed',
               'mcp_tool_failed', 'model_did_not_complete_one_tool_call'}
    reason = str(error) if isinstance(error, (ValueError, RuntimeError)) and str(error) in allowed else 'configuration_or_mcp_or_response_error'
    return {'status': 'failed', 'reason': reason}
