# ROS Package bob_llama_cpp

This ROS package integrates a LLM chat client with function calling capabilities into [`ROS`](https://ros.org) (Robot Operating System). It needs a running `llama.cpp:server` or other compatible completetion API endpoint in order to work. Additionally it makes use of `Huggingface` `Transformers` and `Autotokenizer` library to apply the model specific chat template.

Because this ROS node is so lightweight, it is possible to run multiple LLM clients simultaneously and share the same completion API endpoint. Together with the llama.cpp server, which can also handle parallel requests and shared context, many interesting things like chaining of LLMs can be done. The LLM-ROS node also provides numerous interfaces for further NL processing.

**Following features are available in this package**
* Configurable 
    * chat history
    * system prompt
    * function calling
    * initial conversation
    * HF model_id
    * and other known LLM parameters
    * rich debug output
* ROS Topic interfaces
    * llm_in input String topic
    * llm_out output String topic
    * llm_generator output String topic
    * llm_sentence output String topic
    * dialog JSON role/assistant pair output String topic
    * json JSON structured Metadata output String topic
* Dynamic reconfigure GUI to change parameters at runtime
* String Topic IO Terminal to send or monitor LLM conversations
* Can be started as normal ROS node or as Lifecycle node
* ...

**Example of a running chat client with a connected terminal to communicate with the LLM ROS Node**

![Running chat client with a connected terminal to communicate with the LLM](data:image/png;base64,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)


## Starting a llama.cpp Server
To start a [llama.cpp](https://github.com/ggerganov/llama.cpp) server different prebuild docker container are available with and without cuda support.

Container overview:
* [https://github.com/ggml-org/llama.cpp/pkgs/container/llama.cpp](https://github.com/ggml-org/llama.cpp/pkgs/container/llama.cpp)

A lot of llama compatible models can be found and downloaded from [Huggingface Website](https://huggingface.co/models)\
Look for GGUF models. The higher the quantization, the better function calls work. If a Nvidia GPU should be used with a `Docker` container additionally the `Nvidia Cuda Toolkit` need to be installed.

**Related Links**

* [https://github.com/ggerganov/llama.cpp/blob/master/examples/server/README.md](https://github.com/ggerganov/llama.cpp/blob/master/examples/server/README.md)
* [https://linuxconfig.org/setting-up-nvidia-cuda-toolkit-in-a-docker-container-on-debian-ubuntu](https://linuxconfig.org/setting-up-nvidia-cuda-toolkit-in-a-docker-container-on-debian-ubuntu)
* [https://huggingface.co/models](https://huggingface.co/models)

**Good working models including function calling support**

   * [https://huggingface.co/mistralai/Mistral-Small-24B-Instruct-2501](https://huggingface.co/mistralai/Mistral-Small-24B-Instruct-2501)
   * [https://huggingface.co/mistralai/Mistral-7B-Instruct-v0.3](https://huggingface.co/mistralai/Mistral-7B-Instruct-v0.3)
   * [https://huggingface.co/NousResearch/Hermes-2-Pro-Llama-3-8B](https://huggingface.co/NousResearch/Hermes-2-Pro-Llama-3-8B)


```bash
# docker example to start a llama.cpp server with cuda support
# two completition calls can be handled parallel, each with an own context size of 8192k 
sudo docker run --gpus all -p 8000:8080 \
    -v ./models:/models \
    ghcr.io/ggerganov/llama.cpp:server-cuda \
    -m /models/Mistral-Small-24B-Instruct-2501-Q8_0.gguf \
    --port 8080 --host 0.0.0.0 \
    --n-predict 512 \
    --gpu-layers 12 --threads-http 4 \
    --threads 16 --parallel 2 --ctx-size 16384 \
    --override-kv tokenizer.ggml.add_bos_token=bool:false
```

## ROS Node LLM

This ROS node is basically a client for an OpenAI compatible completition API endpoint. It was tested yet only with an own hosted local [llama_cpp server](https://github.com/ggml-org/llama.cpp/tree/master/examples/server#readme). Other completition API server should work as well.

### Dependencies
There are a couple of dependencies in order to be able to use all available features of this ROS Node. For an overview see [`requirements.txt`](https://github.com/bob-ros2/bob_llama_cpp/blob/main/requirements.txt)

### Starting the LLM client
To run the client a completition endpoint must be available. The default url is: http://localhost:8000

```bash
# launch the LLM client with a terminal to 
# communicate with the completition endpoint
ros2 launch bob_llama_cpp llm.launch.py terminal:=true

# launch with custom nodes configuration
ros2 launch bob_llama_cpp llm.launch.py \
    terminal:=true \
    config_yaml:=my_nodes_config.yaml

# just run the node with configuration file
ros2 run bob_llama_cpp llm \
    --ros-args \
    --params-file my_node_config.yaml

# run the node with parameter and remap topics
ros2 run bob_llama_cpp llm \
    --ros-args \
    -p temperature:=0.9 \
    -p model_id:=mistralai/Mistral-Small-24B-Instruct-2501 \
    -p system_prompt:="<s>[SYSTEM_PROMPT]Answer like a pirate would answer[/SYSTEM_PROMPT]" \
    -r llm_in:=stt_out_topic \
    -r llm_sentence:=tts_in_topic
# want to connect TTS and/or STT? Then check out these packages:
# https://github.com/bob-ros2/rosspeaks
# https://github.com/bob-ros2/bob_whisper_cpp
# or https://github.com/bob-ros2/voskros
```

### Node Parameter

> **Parameter name**: api_key\
> **Type**: string\
> **Description**: The API key, if it's needed to authorize. Environment variable LLM_API_KEY. Default: 'no-key'

> **Parameter name**: api_url\
> **Type**: string\
> **Description**: The API url of the llama.cpp server. Environment variable LLM_API_URL. Default: http://localhost:8000

> **Parameter name**: chat_history\
> **Type**: boolean\
> **Description**: Wether to use a chat history for the conversation or not. Environment variable LLM_CHAT_HISTORY. Default: true

> **Parameter name**: chat_template\
> **Type**: string\
> **Description**: not used yet

> **Parameter name**: eof_indicator\
> **Type**: string\
> **Description**: EOF indicator to use. This indicator will be send after the last token happened. This in usefull if in generator mode to identify the end of the response. Environment variable LLM_EOF_INDICATOR. Default: ''

> **Parameter name**: initial_messages\
> **Type**: string\
> **Description**: Initial conversation JSON list of dictionaries. The list can be provided as file or directly as JSON list. The dicts has to contain the used role and content items. The initial messages are processed using the according chat template, and added to the history. Environment variable LLM_INITIAL_MESSAGES. Default: ''

> **Parameter name**: min_p\
> **Type**: double\
> **Description**: Sets a minimum base probability threshold for token selection. Environment variable LLM_MIN_P. Default: 0.1

> **Parameter name**: model_id\
> **Type**: string\
> **Description**: The base Huggingface model_id to be used. It will be used to apply the according transformers chat template. This will also be used to auto generate a tools call system prompt if the model supports it and if also tools_module parameter was configured. Environment variable LLM_MODEL_ID. Default: ''

> **Parameter name**: n_keep\
> **Type**: integer\
> **Description**: Specify the number of tokens from the initial prompt to retain when the model resets its internal context. By default, this value is set to 0 (meaning no tokens are kept). Use -1 to retain all tokens from the initial prompt. When the system_prompt parameter is set the length will be determined from the token count by using the llama.cpp server tokenizer endoint. Environment variable LLM_N_KEEP. Default: -1

> **Parameter name**: n_predict\
> **Type**: integer\
> **Description**: Set the number of tokens to predict when generating text. Adjusting this value can influence the length of the generated text. Environment variable LLM_N_PREDICT. Default: -1

> **Parameter name**: prompt\
> **Type**: string\
> **Description**: Prompt format. Environment variable LLM_PROMPT. Default: {0}\n

> **Parameter name**: repeat_last_n\
> **Type**: integer\
> **Description**: Last n tokens to consider for penalizing repetition. Environment variable LLM_REPEAT_LAST_N. Default: 64, 0 = disabled, -1 = ctx-size

> **Parameter name**: repeat_penalty\
> **Type**: double\
> **Description**: Control the repetition of token sequences in the generated text. Environment variable LLM_REPEAT_PENALTY. Default: 1.0, 1.0 = disabled.

> **Parameter name**: stop_tokens\
> **Type**: string array\
> **Description**: If one of the stop tokens are received in the llm input topic a running generator will be aborted. Environment variable LLM_STOP_TOKENS. Default: 'stop shutup'

> **Parameter name**: system_prompt\
> **Type**: string\
> **Description**: System prompt to set. If provided the n_keep parameter will be set automatically as well and if the context size exceeds llama.cpp will try to keep this system prompt. Further details how n_keep works can be found in the llama.cpp server documentation. Environment variable LLM_SYSTEM_PROMPT. Default: ''

> **Parameter name**: temperature\
> **Type**: double\
> **Description**: Adjust the randomness of the generated text. Environment variable LLM_TEMPERATURE. Default: 0.1

> **Parameter name**: tools_module\
> **Type**: string\
> **Description**: Path to the tools function python script. The tool functions and their description need to follow the google standard in order to work with HF chat template. Also the type definitions are important in order to produce later the tool call dict. All contained global python functions are treated as potential tool calls. See an example in the config folder of this package. Environment variable LLM_TOOLS_MODULE. Default: ''

> **Parameter name**: top_k\
> **Type**: integer\
> **Description**: Limit the next token selection to the K most probable tokens. Environment variable LLM_TOP_K. Default: 40

> **Parameter name**: top_p\
> **Type**: double\
> **Description**: Limit the next token selection to a subset of tokens with a cumulative probability above a threshold P. Environment variable LLM_TOP_P. Default: 0.9


### Subscribed Topics

> ~llm_in (std_msgs/String)\
LLM input.

### Published Topics

> ~llm_out (std_msgs/String)\
LLM output. The whole message will be published when the generator has finished.

> ~llm_generator (std_msgs/String)\
LLM generator output. Each single token is published to the topic.

> ~llm_sentence (std_msgs/String)\
LLM output aggregated as sentence or sub sentence. A message with content EOF indicates the end of the generator output. Separator list for the sentence can be set with help of the environment variable `SENTENCE_DELIMETER`, the default is ".:,;!?-*\n\t".

> ~dialog (std_msgs/String)\
Output as the known user/assistent conversation pair as JSON array.

> ~json (std_msgs/String)\
Output of a user/assistent conversation pair in structured Metadata format as below. This can be used for example to store structured data into a database. This meta data structure will be used also by other from Bob's ROS packages.  
```Python
{
    "metadata": [
        {"key": "stamp", "value": float("%d.%09d" 
            % (header.stamp.sec, header.stamp.nanosec))},
        {"key": "frame_id", "value": header.frame_id},
        {"key": "tags", "value": ["llm","dialog",user,header.frame_id]},
        {"key": "type", "value": "dialog"},
        {"key": "user", "value": user},
    ],
    "data": dialog
}
```

## Function Calling and Built-in Tool Functions

### How it works
If the used model is ready for function calling and the transformers `model_id` Autotokenizer version supports it, a Python script with functions can be provided. See also parameter `tools_module` for further details.

General tool call documentation from `Huggingface`:
* [https://huggingface.co/blog/unified-tool-use](https://huggingface.co/blog/unified-tool-use)
* [https://huggingface.co/docs/hugs/guides/function-calling](https://huggingface.co/docs/hugs/guides/function-calling)

#### Minimal example for a custom tools module file
```Python
# Mandatory import for every tool module file.
# This 'prompt_tools' will later be replaced with the 
# 'prompt_tools' instance from the calling LLM ROS Node.
# This holds the available tool functions and the tool  
# results which are later used by the LLM
from bob_llama_cpp import prompt_tools

# define a function with a comment block which follow the Google standard
# as better the description is as better it will work later with the model
# also typing should be used for the parameters to work properly
# as less parameter are used as better it will work
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
```

Once the custom tools module is configured the LLM ROS Node will  import it at startup and will load each python function into a tools list for later use.

The user queries together with the tools list are now converted into the model specific format with help of the Huggingface Autotekenizer apply_chat_template function. That output will now be send with the completition request to the model.

When the model decides to use one or more of the available tools it will output the tool calls in JSON format. This output will then be feeded into the following code snippet. 

```python
    if prompt_tools.detect_and_process_tool_calls(output):
        self.queue.append(prompt_tools.generate_tool_results())
```

If one or more function calls were detected it will be executed accordingly and the result will be directly feeded back into the LLM as a new completion request (the tool result is at this point still not visible to the user of the LLM). The model will now take into account these results to formulate a final answer. 


### Available tool functions

Usefull example tool functions can be found in module [`tools_functions.py`](https://github.com/bob-ros2/bob_llama_cpp/blob/main/bob_llama_cpp/tools_functions.py) contained in this repository. Some of them need further configuration in order to work. When done configure the ROS parameter `tools_module` with the full path of the script to use the tools.

#### def search_internet(query: str, limit: int=3)
* This tool makes use of Googles custom-search API. To get free access up to 100 calls per day, or more see:
    * [https://developers.google.com/custom-search/v1/introduction/](https://developers.google.com/custom-search/v1/introduction/)
* Dependencies: requests
* Environment variables, see [`tools_functions.py`](https://github.com/bob-ros2/bob_llama_cpp/blob/main/bob_llama_cpp/tools_functions.py) for details:
    * GOOGLE_SEARCH_URL

#### def grep_url(url: str, filter: str=None)
* This tool calls a webpage with a random useragent to be not detected as a Bot, transforms visible text and links into text.
* Dependencies: BeautifulSoup, getuseragent, requests

#### def remember(context: str, limit: int=3)
* Requirement: A running Qdrant Vector DB with data embedded with the given model in the given collection.
    * [https://qdrant.tech/](https://qdrant.tech/)
* Dependencies: QdrantClient
    * [https://python-client.qdrant.tech/](https://python-client.qdrant.tech/)
* Environment variables and default values
    * TOOL_QDRANT_URL=http://localhost:6333
    * TOOL_QDRANT_MODEL=nomic-ai/nomic-embed-text-v1
    * TOOL_QDRANT_COLLECTION=memo_embedder

#### def topic_list(filter: str='')
* Dependencies: None


## ROS Node TERMINAL
With this Ros Node it is possible to sent directly String topic messages to the LLM. The generator output of the LLM node can also be received and displayed in realtime. The input field can optionally be turned off if needed.
The Ros node can also receive input from stdin which will also be displayed in the text area.

### Dependencies
The required QT5 libraries should already exist if ROS is installed. If missing use below installation to get them.
```bash
sudo apt-get install python3-pyqt5
```

### Usage
```bash
# run frameless terminal window using ROS parameter
ros2 run bob_llama_cpp terminal --ros-args -p frameless:=true -p geometry:=[300,300,600,480]

# show window on another display
ros2 run bob_llama_cpp terminal --ros-args -p display:=1 -p geometry:=[300,300,600,480]
```

### Node Parameter

> ~display\
  Type: integer\
  Display number where to show the window. -1 = default display.\
  Default: -1

> ~fontname\
  Type: string\
  Window fontname.\
  Default: courier

> ~fontsize\
  Type: integer\
  Window fontsize.\
  Default: 12

> ~frameless\
  Type: boolean\
  Switch off window caption.\
  Default: false

> ~geometry\
  Type: integer array\
  Window geometry. [x, y, with, height]

> ~margin\
  Type: integer array\
  Window inner margin. [left, top, right, bottom]\
  Default: [10,0,0,0]

> ~input\
  Type: boolean\
  Enables or disables the text input field.\
  Default: true

> ~opacity\
  Type: double\
  Window opacity. 1.0 = fully visible.\
  Default: 1.0

> ~stylesheet\
  Type: string\
  Stylesheet qss of PlainText area.\
  Default: background-color: #000000; color: #f0f0f0;

> ~title\
  Type: string\
  Title of window.\
  Default: GPT4ALL Terminal

> ~line_count\
  Type: string\
  Maximum line count in the text area. 0 = unlimited.\
  If the number exceeds the lines are removed from the top.
  Default: 0

### Subscribed Topics

> ~topic_in (std_msgs/String)\
Read input data from topic and output it in the terminal text area.

> ~topic_in_cr (std_msgs/String)\
Same as `~topic_in` but in addition output it in the terminal text area with a trailing \n

### Published Topics

> ~topic_out (std_msgs/String)\
Publish to output topic. Parameter `~input` must be truerin order to be able to see and use the input field.

