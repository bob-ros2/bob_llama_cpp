#!/usr/bin/env python3
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
import sys
import logging
import time
import re
import requests
import json
from threading import Thread

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_msgs.msg import Header
from bob_llama_cpp.lnode import LNode
from bob_llama_cpp import prompt_tools

# this ros node needs a running llama.cpp:server or other compatible completetion API endpoint
# https://github.com/ggerganov/llama.cpp/blob/master/examples/server/README.md
# docker example with cuda support:
# sudo docker run --gpus all -p 8000:8080 -v ./models:/models ghcr.io/ggerganov/llama.cpp:server-cuda -m /models/Mistral-7B-Instruct-v0.3.fp16.gguf --port 8080 --host 0.0.0.0 -n 512 --gpu-layers 12 --threads-http 8 --threads 16 --parallel 2 --ctx-size 8192--override-kv tokenizer.ggml.add_bos_token=bool:false

if os.getenv('LLM_LIFECYCLE_NODE', default=False):
    class BaseNode(LNode):
        """Lifecycle ROS Node"""
else:
    class BaseNode(Node):
        """Standard ROS Node"""

class LlmNode(BaseNode):
    """
    LLM ROS Node wrapper
    """
    def __init__(self):
        super().__init__('llm')

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() \
                    == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.declare_parameters(
            namespace='',
            parameters=[

                ('prompt', os.getenv('LLM_PROMPT', "{0}\n"),
                ParameterDescriptor(description=
                'Prompt format, default: {0}\\n')),

                ('system_prompt', os.getenv('LLM_SYSTEM_PROMPT', ''), 
                ParameterDescriptor(description=
                'System prompt to set. If provided the n_keep parameter will '
                'be set automatically as well and if the context size exceeds '
                'llama.cpp will try to keep this system prompt. Further details '
                'how n_keep works can be found in the llama.cpp server documentation. '
                'Default: \'\'')),

                ('initial_messages', os.getenv('LLM_INITIAL_MESSAGES', ''), 
                ParameterDescriptor(description=
                'Initial conversation JSON list of dictionaries. '
                'The list can be provided as file or directly as JSON list. '
                'The dicts has to contain the used role and content items. '
                'The initial messages are processed using the according chat template, '
                'and added to the history. Default: \'\'')),

                ('chat_history', os.getenv('LLM_CHAT_HISTORY', 'true') == 'true', 
                ParameterDescriptor(description=
                'Wether to use a chat history for the conversation or not, default: true')),

                ('api_url', os.getenv('LLM_API_URL', 'http://localhost:8000'), 
                ParameterDescriptor(description=
                'The API url of the llama.cpp server, default: http://localhost:8000')),

                ('temperature', float(os.getenv('LLM_TEMPERATURE', '0.1')), 
                ParameterDescriptor(description=
                'Adjust the randomness of the generated text, default: 0.1')),

                ('n_predict', int(os.getenv('LLM_N_PREDICT', '-1')), 
                ParameterDescriptor(description=
                'Set the number of tokens to predict when generating text. '
                'Adjusting this value can influence the length of the generated text. Default: -1')),

                ('chat_template', os.getenv('LLM_CHAT_TEMPLATE', ''), 
                ParameterDescriptor(description=
                'not used yet')),

                ('stop_tokens', os.getenv('LLM_STOP_TOKENS', 'shutup').split(), 
                ParameterDescriptor(description=
                'If one of the stop tokens are received in the llm input topic '
                'a running generator will be aborted.')),

                ('top_k', int(os.getenv('LLM_TOP_K', '40')), 
                ParameterDescriptor(description=
                'Limit the next token selection to the K most probable tokens, default: 40')),

                ('top_p', float(os.getenv('LLM_TOP_P', '0.9')), 
                ParameterDescriptor(description=
                'Limit the next token selection to a subset of tokens '
                'with a cumulative probability above a threshold P, default: 0.9')),

                ('min_p', float(os.getenv('LLM_MIN_P', '0.05')), 
                ParameterDescriptor(description=
                'Sets a minimum base probability threshold for token selection, default: 0.1')),

                ('repeat_penalty', float(os.getenv('LLM_REPEAT_PENALTY', '1.0')), 
                ParameterDescriptor(description=
                'Control the repetition of token sequences in the generated text, '
                'default: 1.0, 1.0 = disabled.')),

                ('repeat_last_n', int(os.getenv('LLM_REPEAT_LAST_N', '64')), 
                ParameterDescriptor(description=
                'Last n tokens to consider for penalizing repetition, '
                'default: 64, 0 = disabled, -1 = ctx-size')),

                ('n_keep', int(os.getenv('LLM_N_KEEP', '-1')),
                ParameterDescriptor(description=
                'Specify the number of tokens from the initial prompt to retain '
                'when the model resets its internal context. '
                'By default, this value is set to 0 (meaning no tokens are kept). '
                'Use -1 to retain all tokens from the initial prompt. '
                'When the system_prompt parameter is set the length will be determined '
                'from the token count by using the llama.cpp server tokenizer endoint. Default: -1')),

                ('model_id', os.getenv('LLM_MODEL_ID', ''), 
                ParameterDescriptor(description=
                'The base Huggingface model_id to be used. '
                'It will be used to apply the according transformers chat template. '
                'This will also be used to auto generate a tools call system prompt if the model supports it and '
                'if also tools_module parameter was configured. Default: \'\'')),

                ('tools_module', os.getenv('LLM_TOOLS_MODULE', ''), 
                ParameterDescriptor(description=
                'Path to the tools function python script. The tool functions and their '
                'description need to follow the google standard in order to work with HF chat template. '
                'Also the type definitions are important in order to produce later the tool call dict. '
                'All contained global python functions are treated as potential tool calls. '
                'See an example in the config folder of this package. Default: \'\'')),

                ('eof_indicator', os.getenv('LLM_EOF_INDICATOR', ''), 
                ParameterDescriptor(description=
                'EOF indicator to use. This indicator will be send after the last token happened. '
                'This in usefull if in generator mode to identify the end of the response. Default: \'\''))
            ])

        self.system_prompt = None
        self.tools_module = None
        self.n_keep = -1

        self.sub_in = None
        self.sentence = ''
        self.session_thread = None
        self.queue = list()
        self.history = list()

        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            history=qos.QoSHistoryPolicy.KEEP_ALL,
            depth=1000)

        self.pub_generator = self.create_publisher(
            String, 'llm_generator', qos_profile)

        self.pub_out = self.create_publisher(
            String, 'llm_out', 1000)

        self.pub_sentence = self.create_publisher(
            String, 'llm_sentence', 1000)

        self.pub_dialog = self.create_publisher(
            String, 'llm_dialog', 1000)

        if not os.getenv('LLM_LIFECYCLE_NODE', default=False):
            self.configure()

    def configure(self) -> None:
        """
        Start/restart initialization of the node
        """
        self.gpt_abort = False
        self.running = True
        self.history = list()
        self.sentence = ''

        # import tool functions if provided
        module_name = os.path.basename(self.get_parameter(
            'tools_module').value)[:-3]
        if module_name:
            self.tools_module = prompt_tools.import_module_from_path(
                module_name, self.get_parameter(
                    'tools_module').value)
            # make sure imported module prompt_tools lib is in the the same scope as ours
            if hasattr(self.tools_module, 'prompt_tools'):
                self.tools_module.prompt_tools = prompt_tools
            else:
                self.get_logger().fatal(
                    f"configure: module '{module_name}' "
                    "does not contain imported bob_llama_cpp.prompt_tools. Exiting node.")
                sys.exit(1)

        # set system_prompt
        self.system_prompt = self.get_parameter(
            'system_prompt').value

        # load initial messages if existing
        initial_messages = self.get_parameter('initial_messages').value
        if initial_messages:
            self.get_logger().info("loading initial_messages...")
            try:
                conversation = json.loads(initial_messages)
                self.history += conversation
            except ValueError:
                try:
                    self.get_logger().debug(
                        "parsing initial_messages as json failed, trying to load as file")
                    with open(initial_messages) as file:
                        conversation = json.loads(file.read())
                        self.history += conversation
                except:
                    self.get_logger().warn(
                        f'configure: provided param initial_messages is neither json parsable '
                        f'nor a readable json file. Continue without initial_messages: '
                        f'{initial_messages}')

        # if there is a system_prompt set n_keep accordingly
        if self.system_prompt:
            self.n_keep = len(self.tokenize(self.system_prompt))
            self.get_logger().info(
                f"set n_keep to system_prompt token count of {self.n_keep}")

        self.get_logger().info(
            "system prompt: %s" % (self.system_prompt or 'empty'))

        # run llm loop
        self.session_thread = Thread(target=self.session)
        self.session_thread.start()

        # start listening for input
        self.sub_in = self.create_subscription(
            String, 'llm_in', 
            self.llm_in_callback, 10)

    def destroy(self) -> None:
        """
        Stop llm thread, destroy subscriber
        """
        self.gpt_abort = True
        self.running = False

        if self.session_thread:
            self.session_thread.join()
            self.session_thread = None

        if self.sub_in:
            self.destroy_subscription(self.sub_in)
            self.sub_in = None

        if self.tools_module:
            del self.tools_module
            self.tools_module = None

    def llm_in_callback(self, msg: String) -> None:
        if msg.data in self.get_parameter(
            'stop_tokens').value:
            self.gpt_abort = True
        else:
            self.queue.append(msg.data)
        self.get_logger().debug(msg.data)

    def tokenize(self, text: str, add_special: bool=False) -> list:
        """
        Tokenize text and return the tokens list.
        """
        try:
            with requests.Session().post(
                self.get_parameter('api_url').value+'/tokenize', 
                headers = {"Content-Type": "application/json"},
                json = {
                    'content': text, 
                    'add_special': add_special}) as resp:
                self.get_logger().debug(f'tokenize: {resp.json()}')
                return resp.json()['tokens']
        except Exception as e:
            self.get_logger().error(f'tokenize: exception: {e}')
        return []

    def jsonfy(self, dialog, prompt_user) -> str:
        """
        Format a json object from an array with user/assistant messages.
        The prompt_user should match ^username:.*
        """
        header = Header() 
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_name()
        if prompt_user.startswith('<s>[TOOL_CALLS]'):
            user = 'tool'
        elif re.match(r'^[^ :]+:.*', prompt_user):
            user = re.sub(r'^([^ :]+):.*', r'\1', prompt_user)
        else: user = "user"

        j = json.dumps({
            "metadata": [
                {"key": "stamp", "value": float("%d.%09d" 
                    % (header.stamp.sec, header.stamp.nanosec))},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "tags", "value": ["llm","dialog",user,header.frame_id]},
                {"key": "type", "value": "dialog"},
                {"key": "user", "value": user},
            ],
            "data": dialog
        })

        self.get_logger().debug(f'jsonfy: {j}')
        return j

    def generate(self, prompt: str) -> str:
        """
        Generate streaming completion
        """

        body = {
            "stream":         True,
            "prompt":         prompt,
            "n_predict":      self.get_parameter("n_predict").value,
            "temperature":    self.get_parameter("temperature").value,
            "top_k":          self.get_parameter("top_k").value,
            "top_p":          self.get_parameter("top_p").value,
            "min_p":          self.get_parameter("min_p").value,
            "repeat_penalty": self.get_parameter("repeat_penalty").value,
            "repeat_last_n":  self.get_parameter("repeat_last_n").value,
            "n_keep":         self.n_keep if self.n_keep > -1 \
                                  else self.get_parameter('n_keep').value
        }

        try:
            with requests.Session().post(
                self.get_parameter('api_url').value+'/completion', 
                headers = {"Content-Type": "application/json"}, 
                stream = True, 
                json = body) as resp:
                for line in resp.iter_lines():
                    line = line.decode('utf-8')
                    if line.startswith('data: '):
                        data = json.loads(line[6:].strip())
                        yield data['content']
        except Exception as e:
            self.get_logger().error(f"generate: exception: {e}")

    def session(self) -> None:
        """
        Process incoming queue messages.
        """
        while self.running:
            time.sleep(0.005)
            if len(self.queue):

                output = ''
                prompt = self.queue.pop(0)
                if not prompt:
                    self.get_logger().warn(
                        "the last prompt is None and will be skipped")
                    continue
                dialog = [{'role': 'user', 'content': prompt}]

                if prompt.startswith('<s>'):
                    prompt_message = prompt
                    #self.token_handler(prompt_message)
                    self.print(prompt_message)
                else:
                    prompt_message = self.get_parameter(
                        'prompt').value.format(prompt)
                    self.token_handler(prompt_message)
                    self.print(prompt_message)

                    if self.get_parameter("chat_history").value:
                        conversation = self.history \
                            + [{'role':'user','content':prompt_message}]
                        prompt_message = prompt_tools.apply_chat_template(
                            conversation, self.get_parameter('model_id').value)
                    else:
                        prompt_message = prompt_tools.apply_chat_template(
                            [{'role':'user','content':prompt_message}],
                            self.get_parameter('model_id').value)

                    if self.system_prompt:
                        prompt_message = self.system_prompt + prompt_message

                    self.get_logger().debug(
                        f"prompt_message: \033[0;32m{prompt_message}\033[0m")

                for token in self.generate(
                    f"{prompt_message}"):

                    if self.gpt_abort:
                        self.gpt_abort = False
                        break
                    self.print(token)
                    self.token_handler(token)
                    output += token

                output = output.strip(" \n")
                self.print("\n")
                self.token_handler("\n")
                self.token_handler('EOF')
                self.publish(output, self.pub_out)

                dialog.append({'role': 'assistant', 'content': output})

                self.history += dialog
                self.publish(self.jsonfy(dialog, prompt_message), self.pub_dialog)

                logging.debug(
                    f"\033[0;31mself.history:\n{json.dumps(self.history,indent=2)}\n\033[0m")

                if prompt_tools.detect_and_process_tool_calls(output):
                    self.queue.append(prompt_tools.generate_tool_results())

    def token_handler(self, token: str) -> None:
        """
        Publishes to the generator and sentence publishers.
        """
        SENTENCE_DELIMETER = \
            os.getenv('SENTENCE_DELIMETER', default=".:,;!?-*\n\t")

        if token == 'EOF':
            eof_indicator = self.get_parameter(
                'eof_indicator').value
            if eof_indicator:
                self.publish(self.sentence+eof_indicator, self.pub_sentence)
                self.publish(eof_indicator, self.pub_generator)
            else: 
                self.publish(self.sentence, self.pub_sentence)
            self.sentence = ''
            return
        else:
            self.publish(token, self.pub_generator)

        for c in token:
            self.sentence += c
            if c in SENTENCE_DELIMETER:
                self.publish(self.sentence, self.pub_sentence)
                self.sentence = ''

    def publish(self, data: str, pub) -> None:
        """
        Publish data to given String topic.
        """
        pub.publish(String(data=data))
        self.get_logger().debug(f"Published: {data}")

    def print(self, s: str):
        """
        Print to stdout.
        """
        print(s, end="", flush=True)


def main(args=None):
    rclpy.init(args=args)
    n = LlmNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
