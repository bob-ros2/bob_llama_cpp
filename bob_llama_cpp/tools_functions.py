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

# The tool functions and their description need to follow the google standard in order to work with HF chat template.
# Also the type definitions are important here to produce later the tool call dict.

import os
import re
import logging
import requests
import string
import random
from bob_llama_cpp import prompt_tools

def search_internet(query: str, limit: int=3):
    """
    Search in the internet

    Args:
        query: The query to search for in the internet
        limit: Optional count of search results to return. The maximum count is 20, default 3
    """
    logging.info(f"""called search_internet("{query}")""")
    id_tool_call = ''.join(random.choices(string.ascii_letters+string.digits, k=9))
    results = "Link: dummy.com Snippet: This is a dummy search result\nLink: another.com Snippet: Thats another result "
    
    # Here google customsearch will be used, each user has 100 free API search queries per day.
    # This can be activated for free in the google account settings.
    # https://developers.google.com/custom-search/v1/introduction/

    try:
        limit = limit if limit < 20 else 20
        dummy_url = "https://customsearch.googleapis.com/customsearch/v1?cx=mycxid12345&key=mytoken12345&q="
        base_url = os.getenv('GOOGLE_SEARCH_URL', dummy_url)
        url = requests.utils.requote_uri(f"{base_url}{query}")
        response = requests.get(url)
        j = response.json()
        results = "\n".join([
            f"Link: {i['link']}\nSnippet: {i['snippet']}" 
            for n,i in enumerate(j['items']) if n<limit
        ])
    except Exception as e: 
        results = f"No data found! {e}"
    
    prompt_tools.tool_calls.append((
        {"name": "search_internet", "arguments": {"query": f"{query}"}, "id": f"{id_tool_call}"}, 
        {"call_id": f"{id_tool_call}", "content": f"{results}"}))

def topic_list(filter: str=''):
    """
    Get the available ROS topic list

    Args:
        filter: text to look for within the topic list
    """
    logging.info(f"""called topic_list("{filter}")""")
    id_tool_call = ''.join(random.choices(string.ascii_letters+string.digits, k=9))

    import subprocess

    result = subprocess.run(['ros2','topic','list'], 
        shell=False, capture_output=True, 
        text=True, env=os.environ.copy())
    logging.info(result.stdout)

    if result.stderr:
        logging.error(result.stderr)

    result = ''.join(line for line in result.stdout.splitlines(True) if filter in line)
    prompt_tools.tool_calls.append((
        {"name": "topic_list", "arguments": {"filter": f"{filter}"}, "id": f"{id_tool_call}"}, 
        {"call_id": f"{id_tool_call}", "content": result}))

def grep_url(url: str, filter: str=None):
    """
    Grep content from a resource behind an url address or a link

    Args:
        url: URL address link of the resource to grep
        filter: An optional filter what to grep from the resource
    """
    logging.info(f"""called grep_url("{url}","{filter}")""")
    id_tool_call = ''.join(random.choices(string.ascii_letters+string.digits, k=9))

    from bs4 import BeautifulSoup
    from getuseragent import UserAgent
    import requests

    try:
        ue = UserAgent("all", requestsPrefix=True).Random()
        logging.info(f"GET using user agent: {ue}")
        response = requests.get(url, headers=ue)
        response.raise_for_status()
        soup = BeautifulSoup(response.text, 'html.parser')
        text = "Content from the resource behind url:\n" + soup.get_text()
        links = soup.find_all('a')
        link_list = ''
        for link in links:
            href = link.get('href')
            if href and link.text and href.startswith('https'):
                href = href.strip("\n ")
                link_text = link.text.strip("\n ")
                link_list += f"[{link_text}]({href})\n"
        if link_list: 
            text += f"\nLinks contained in the resource behind url:\n{link_list}"
        text = re.sub(r"(.*)\n\n+(.*)", r"\1\n\n\2", text)

    except requests.exceptions.HTTPError as e:
        text = f"An error occured, the provided URL could not be opened! {e}"

    prompt_tools.tool_calls.append((
        {"name": "grep_url", "arguments": {"url": f"{url}","filter": f"{filter}"}, "id": f"{id_tool_call}"}, 
        {"call_id": f"{id_tool_call}", "content": text}))


qdrant_client = None

def remember(context: str, limit: int=3):
    """
    Remember and recall past conversations, thoughts, incidents similar to the given context.

    Args:
        context: The context to look for similarity in the memories
        limit: Optional count of memories to return. The maximum count is 50 memories, default 3
    """
    logging.info(f"""called remember("{context}",{limit})""")
    id_tool_call = ''.join(random.choices(string.ascii_letters+string.digits, k=9))

    limit = limit if limit < 50 else 50

    from qdrant_client import QdrantClient
    from qdrant_client.models import Filter, FieldCondition, MatchValue
    global qdrant_client

    try:
        url = os.getenv('TOOL_QDRANT_URL', 'http://localhost:6333')
        query_model = os.getenv('TOOL_QDRANT_MODEL', 'nomic-ai/nomic-embed-text-v1')
        collection = os.getenv('TOOL_QDRANT_COLLECTION', 'memo_embedder')

        qdrant_client = qdrant_client or QdrantClient(url, prefer_grpc=True)
        qdrant_client.set_model(query_model)

        results = qdrant_client.query(
            collection_name = collection,
            query_text = context,
            #query_filter = Filter(
            #    must=[  # These conditions are required for search results
            #        FieldCondition(
            #            key="metadata[].key",
            #            match=MatchValue(value="user_name")
            #        ),
            #        FieldCondition(
            #            key="metadata[].value",                        
            #            match=MatchValue(value=user_name)
            #        )
            #    ]
            #),
            limit = limit)

        docs = "\n".join(r.document for r in results)
        result = f"Prompt: {context.strip()}\nMost {limit} similar results:\n{docs.strip()}\n"
    except Exception as e:
        result = "exception during remember tool call"
        logging.error(f"{result}: {e}")

    logging.debug(f"Found {len(results)} results:\n{result}")
    
    prompt_tools.tool_calls.append((
        {"name": "remember", "arguments": {"context": f"{context}","limit": limit}, "id": f"{id_tool_call}"}, 
        {"call_id": f"{id_tool_call}", "content": result}))
