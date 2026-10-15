# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small client for an Ollama server, shared by the *_ollama nodes."""

import json
import os
import re
import urllib.request

from openai import OpenAI

DEFAULT_HOST = 'http://localhost:11434'
# Fits in a 4 GB GPU. Bigger models (e.g. qwen2.5:7b, gemma3:12b) answer better
DEFAULT_MODEL = 'qwen2.5:3b'


def normalize_host(host):
    """Turn an OLLAMA_HOST-like value ('server', 'server:11434', 'http://...') into a URL."""
    host = host.strip().rstrip('/')
    if not host:
        return DEFAULT_HOST
    if '://' not in host:
        host = 'http://' + host
    # Default Ollama port when only the host name is given
    if not re.search(r':\d+$', host.split('://', 1)[1]):
        host += ':11434'
    return host


class OllamaLLM:
    """Chat with a model served by Ollama through its OpenAI-compatible API."""

    def __init__(self, logger, host, model, timeout=60.0, keep_alive=''):
        # Empty host parameter: use OLLAMA_HOST, as the ollama command line does
        self.host = normalize_host(host or os.getenv('OLLAMA_HOST', ''))
        self.model = model or DEFAULT_MODEL
        # How long the server keeps the model in memory after a request ('10m', '1h',
        # '-1' = forever). Empty: the server default (OLLAMA_KEEP_ALIVE, 5 minutes)
        self.extra_body = {'keep_alive': keep_alive} if keep_alive else None
        self.logger = logger
        # Ollama ignores the API key, but the OpenAI client needs one
        self.client = OpenAI(base_url=self.host + '/v1', api_key='ollama', timeout=timeout)

    def check(self):
        """Log whether the server is reachable and has the model. Return True if so."""
        try:
            with urllib.request.urlopen(self.host + '/api/tags', timeout=5) as resp:
                tags = json.load(resp)
        except Exception as e:
            self.logger.error(
                f'Ollama server not reachable at {self.host} ({e}). Start it with '
                f"'ollama serve' or set the 'host' parameter / OLLAMA_HOST.")
            return False

        names = [m['name'] for m in tags.get('models', [])]
        # 'qwen2.5:3b' is listed as is; a model without tag is listed as 'name:latest'
        if self.model not in names and f'{self.model}:latest' not in names:
            self.logger.error(
                f"Model '{self.model}' not found in {self.host}. Download it with "
                f"'ollama pull {self.model}'. Available: {', '.join(names) or 'none'}")
            return False

        self.logger.info(f"Ollama server at {self.host}, model '{self.model}'")
        return True

    def preload(self):
        """Load the model in memory now, so that the first request is not slow."""
        self.logger.info(f"Loading '{self.model}' in Ollama...")
        try:
            self.chat('', 'hi', max_tokens=1)
            self.logger.info(f"Model '{self.model}' loaded")
        except Exception as e:
            self.logger.warn(f'Could not preload the model: {e}')

    def chat(self, system_prompt, user_prompt, max_tokens=150):
        messages = []
        if system_prompt:
            messages.append({'role': 'system', 'content': system_prompt})
        messages.append({'role': 'user', 'content': user_prompt})

        completion = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=0.0,
            extra_body=self.extra_body,
        )
        return completion.choices[0].message.content or ''
