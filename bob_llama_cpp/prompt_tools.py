#
# Copyright 2024 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import re
import json
import logging
from typing import Any
from inspect import getmembers
from inspect import isfunction
import importlib.util

# The tool functions and their description need to follow the google standard in order to work with HF chat template.
# Also the type definitions are important here to produce later the tool call dict.
# example tool_system_prompt for Mistral-7B-Instruct-v0.3: '<s>[AVAILABLE_TOOLS] [{"type": "function", "function": {"name": "get_current_weather", "description": "Get the current weather", "parameters": {"type": "object", "properties": {"location": {"type": "string", "description": "The city and state, e.g. Rouen, FR"}, "format": {"type": "string", "enum": ["celsius", "fahrenheit"], "description": "The temperature unit to use. Infer this from the users location."}}, "required": ["location"]}}}, {"type": "function", "function": {"name": "search_internet", "description": "Search in the internet", "parameters": {"type": "object", "properties": {"query": {"type": "string", "description": "The query to search for in the internet"}}, "required": ["query"]}}}][/AVAILABLE_TOOLS][INST] You are a helpfull assistant with name BobRos based on model mistralai/Mistral-7B-Instruct-v0.3. Make use the available tools to solve your tasks[/INST] How can I assist?</s>'

tool_calls = []
tool_functions = {}
model_default = "mistralai/Mistral-7B-Instruct-v0.3"
tokenizer = None

def generate_tool_results() -> str:
    """Generate tool result response from tool_calls list.
    If global module var tokenizer is not initialized
    the tool results are generated in lama2 format

    :return: The string with generated tool results.
    Returns None if there were no tool calls to process
    :rtype: str
    """

    global tokenizer, tool_calls, tool_functions, model_default

    if not len(tool_calls): return None

    if tokenizer:
        messages = []

        def remove_key(d, key):
            r = dict(d)
            del r[key]
            return r

        messages.append({"role": "assistant", "tool_calls": [{
                "id":       t[1]['call_id'], 
                "type":     "function", 
                "function": remove_key(t[0], 'id')
            } for t in tool_calls]
        })

        for t in tool_calls:
            messages.append({"role": "tool", 
                "tool_call_id": t[1]['call_id'], 
                "name":         t[0]['name'], 
                "content":      t[1]['content']})

        result_prompt = tokenizer.apply_chat_template(messages, tools=tool_functions.values(), 
            tokenize=False, add_generation_prompt=True)
    else:
        logging.warn(f"tokenizer not initialized, generating tools results in lama2 format")
        result_prompt = '<s>[TOOL_CALLS] ' + json.dumps([t[0] for t in tool_calls]) + '</s>'
        result_prompt += ''.join(['[TOOL_RESULTS] ' + json.dumps(t[1]) + '[/TOOL_RESULTS] ' 
            for t in tool_calls])

    logging.info(f"generate_tool_results: {result_prompt}")

    tool_calls.clear()
    return result_prompt


def parse_tool_calls(
    text: str, 
    calls: list) -> str:
    """Parse text for json arrays, multiple arrays are merged together 
    into the calls argument.

    :param text: The string containing the json array
    :type text: str
    :param calls: The list where array items are added if found a json array
    :type calls: list
    :return: The remaing text without the python array
    :rtype: str
    """

    lbrs = 0
    rbrs = 0
    j = ''
    rest = ''
    for c in text:
        if c == '[': lbrs += 1
        elif c == ']': rbrs  += 1
        if lbrs and lbrs == rbrs:
            lbrs = rbrs = 0
            try: 
                array = json.loads(j+c)
                calls += array
            except Exception as e: 
                logging.debug(
                    f"parse_tool_calls: failed to decode json {j}: {e}")
            j = ''
        elif lbrs: j += c
        else: rest += c
    return rest


def detect_and_process_tool_calls(
    response: str) -> int:
    """Check if tools calls exists in the response. If one or more are found 
    and it matches a tool call it will try to execute it/them.
    If the execution fails an exception will be raised.
    The tool detection works with [] and <tool_call></tool_call> pairs.

    :param response: The text where to look for json function tool calls
    :type response: str
    :return: The count of called tool function
    :rtype: int
    """

    text = response.replace('<tool_call>', '[')
    text = text.replace('</tool_call>', ']')

    if all(c in text for c in '[{}]'):
        global tool_functions
        calls = []
        count = 0
        logging.info(f"detect_and_process_tool_calls: {text}")
        logging.info(f"detect_and_process_tool_calls: Rest: " + parse_tool_calls(text, calls))

        for tool in calls:
            logging.info(f"detect_and_process_tool_calls: tool {tool}")
            if 'name' in tool and 'arguments' in tool:
                try:
                    logging.info(
                        f"execute: {tool['name']}({tool['arguments']})")
                    tool_functions[tool['name']](**tool['arguments'])
                    count += 1
                except Exception as e: 
                    logging.error(
                        f"detect_and_process_tool_calls: exception: {e}")
        return count
    return 0


def import_module_from_path(
    module_name: str, 
    path: str) -> Any:
    """Function to dynamically import a module from file path

    :param module_name: Name of the module
    :type module_name: str
    :param path: Path to python module file
    :type path: str
    :return: The loaded python module
    :rtype: Any
    """

    global tool_functions
    # Create a module spec from the given path
    spec = importlib.util.spec_from_file_location(module_name, path)

    # Load the module from the created spec
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # register all functions as tools
    for fnc in getmembers(module, isfunction):
        tool_functions[fnc[0]] = fnc[1]

    logging.info(f"imported tool_functions: {tool_functions}")

    return module

def apply_chat_template(
    conversation: list = None, 
    model_id: str=None) -> str:
    """Generate a system prompt with the available tools using HF AutoTokenizer apply_chat_template function.
    It takes into account the model specific prompt format.

    :param conversation: Array with role/content dicts, defaults to None
    :type conversation: list, optional
    :param model_id: The Huggingface model_id to be used by the tokenizer, defaults to None
    :type model_id: str, optional
    :return: The generated prompt
    :rtype: str
    """

    from transformers import AutoTokenizer
    global tool_functions, model_default, tokenizer

    model_default = model_id or model_default
    tokenizer = tokenizer or AutoTokenizer.from_pretrained(model_id)

    tools = [fnc[1] for fnc in tool_functions.items()]
    tools = tools if len(tools) else None

    return tokenizer.apply_chat_template(
        conversation,
        tools=tools,
        add_generation_prompt=True,
        tokenize=False)
