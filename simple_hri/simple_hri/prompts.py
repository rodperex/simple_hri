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

"""Prompts shared by every Extract and Yes/No backend, so that all of them ask the same."""

import os
import re

from ament_index_python import get_package_share_directory
import yaml

# Default prompts file (in params/): system prompt and user message of each service
PROMPT_FILE = 'prompts_es.yaml'

# Reasoning models (qwen3, deepseek-r1...) may return their thoughts in the answer
THINK_RE = re.compile(r'<think>.*?</think>', re.DOTALL)


def resolve_prompt_path(prompt_file):
    """Absolute paths are used as is; other names are looked up in the package params/."""
    if os.path.isabs(prompt_file):
        return prompt_file
    return os.path.join(get_package_share_directory('simple_hri'), 'params', prompt_file)


class Prompt:
    """Prompt of one service: system instructions and template of the user message."""

    def __init__(self, path, system, user):
        self.path = path
        self.system = system
        self.template = user

    def user(self, **fields):
        """User message of a request, e.g. prompt.user(interest='bebida', text='...')."""
        return self.template.format(**fields)


def load_prompt(prompt_file, service):
    """Load the prompt of a service ('extract' or 'yesno') from a prompts YAML file."""
    path = resolve_prompt_path(prompt_file)
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    try:
        system = data[service]['system'].strip()
        user = data[service]['user'].strip()
    except (KeyError, TypeError, AttributeError):
        raise ValueError(f"{path} needs '{service}: {{system: ..., user: ...}}'")
    return Prompt(path, system, user)


def clean_answer(answer):
    """Remove reasoning, surrounding quotes and an echoed '->' from a model answer."""
    answer = THINK_RE.sub('', answer or '').strip()
    # Some models repeat the end of the request ("... -> coca cola")
    if '->' in answer:
        answer = answer.rsplit('->', 1)[1]
    return answer.strip().strip('"\'`').strip()


def clean_extracted(answer):
    """Clean an Extract answer: NONE if the model returned nothing."""
    answer = clean_answer(answer)
    if not answer or answer.upper().startswith('NONE'):
        return 'NONE'
    # One value per line or with spaces after ';' -> 'a;b'
    values = [v.strip() for v in re.split(r'[;\n]', answer) if v.strip()]
    return ';'.join(values)


def parse_yesno(answer):
    """Map a Yes/No answer of a model to YES, NO or UNKNOWN."""
    word = re.sub(r'[^A-ZÁÉÍÓÚÑ]', ' ', clean_answer(answer).upper()).split()
    if not word:
        return 'UNKNOWN'
    if word[0] in ('YES', 'SÍ', 'SI'):
        return 'YES'
    if word[0] == 'NO':
        return 'NO'
    return 'UNKNOWN'
