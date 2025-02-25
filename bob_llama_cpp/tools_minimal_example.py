# Mandatory import for every tool module file.
# The 'prompt_tools' will later be replaced with the 
# instance from the calling LLM ROS Node.
# This holds the available tool functions and the tool  
# results which are later used by the LLM
from bob_llama_cpp import prompt_tools

def random_number(maximum: int=10):
    """
    Get a random number

    Args:
        maximum: The maximum value of the generated random number. Default is 10.
    """
    import random, string
    id_tool_call = ''.join(random.choices(string.ascii_letters+string.digits, k=9))

    result = random.randrange(maximum)

    prompt_tools.tool_calls.append((
        {"name": "random_number", "arguments": {"maximum": f"{maximum}"}, "id": f"{id_tool_call}"}, 
        {"call_id": f"{id_tool_call}", "content": result}))
