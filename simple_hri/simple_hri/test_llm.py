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

"""Check the running Extract and Yes/No services with a set of sentences and score them.

Usage: ros2 run simple_hri test_llm [extract|yesno] [es|en]
(both services and Spanish if not given; use the language simple_hri was launched with)
"""

import sys
import time

import rclpy

from simple_hri_interfaces.srv import Extract, YesNo

# (interest, text, accepted answers). Compared in lowercase and with the values sorted.
EXTRACT_ES = [
    ('nombre', 'hola, soy María José', ['maría josé']),
    ('nombre', 'hola, me llamo Ana', ['ana']),
    ('nombre', 'buenas, yo soy Pedro y vengo a por un café', ['pedro']),
    ('nombre', 'mi nombre es José Antonio, encantado', ['josé antonio']),
    ('nombre', 'no te lo pienso decir', ['none']),
    ('bebida', 'me apetece una coca cola bien fría', ['coca cola', 'coca-cola']),
    ('bebida', 'hoy hace buen día', ['none']),
    ('bebida', 'ponme un zumo de naranja y un agua', ['agua;zumo de naranja']),
    ('bebida', 'nada, gracias', ['none']),
    ('bebida', 'un vaso de leche, por favor', ['leche', 'vaso de leche']),
    ('objeto', 'tráeme la taza roja de la cocina', ['taza']),
    ('habitación', 'tráeme la taza roja de la cocina', ['cocina']),
    ('habitación', 've al salón y luego al baño', ['baño;salón']),
    ('comida', 'para cenar quiero pizza con piña', ['pizza con piña']),
    ('comida', 'tengo hambre, me comería unas lentejas', ['lentejas']),
    ('color', 'el coche es verde oscuro', ['verde oscuro', 'verde']),
    ('deporte', 'me gusta jugar al fútbol y al pádel', ['fútbol;pádel']),
    ('persona', 'avisa a mi madre de que llego tarde', ['madre', 'mi madre']),
    ('ciudad', 'nací en Sevilla pero vivo en Madrid', ['madrid;sevilla']),
    ('animal', 'tengo un perro y dos gatos', ['gatos;perro', 'gato;perro']),
]

# (text, expected answer)
YESNO_ES = [
    ('sí, claro', 'YES'), ('ni de broma', 'NO'), ('pues no sé', 'UNKNOWN'), ('vale', 'YES'),
    ('venga, adelante', 'YES'), ('mejor no', 'NO'), ('por supuesto que sí', 'YES'),
    ('no, gracias', 'NO'), ('de acuerdo', 'YES'), ('qué va', 'NO'), ('claro que no', 'NO'),
    ('no estoy seguro', 'UNKNOWN'), ('hmm, bueno, vale', 'YES'), ('nunca', 'NO'),
    ('buenos días', 'UNKNOWN'), ('sí', 'YES'), ('no', 'NO'), ('okay', 'YES'),
    ('ahora no, quizá luego', 'NO'), ('me parece bien', 'YES'),
]

EXTRACT_EN = [
    ('name', "hi, I'm Mary Jane", ['mary jane']),
    ('name', 'hello, my name is Anna', ['anna']),
    ('name', "hey, I'm Peter and I came for a coffee", ['peter']),
    ('name', 'my name is John Paul, nice to meet you', ['john paul']),
    ('name', "I won't tell you", ['none']),
    ('drink', "I'd like a really cold coke", ['coke']),
    ('drink', "it's a nice day today", ['none']),
    ('drink', 'give me an orange juice and a water', ['orange juice;water']),
    ('drink', 'nothing, thanks', ['none']),
    ('drink', 'a glass of milk, please', ['milk', 'glass of milk']),
    ('object', 'bring me the red cup from the kitchen', ['cup']),
    ('room', 'bring me the red cup from the kitchen', ['kitchen']),
    ('room', 'go to the living room and then to the bathroom', ['bathroom;living room']),
    ('food', 'for dinner I want pizza with pineapple', ['pizza with pineapple']),
    ('food', "I'm hungry, I'd eat some lentils", ['lentils']),
    ('color', 'the car is dark green', ['dark green', 'green']),
    ('sport', 'I like playing football and tennis', ['football;tennis']),
    ('person', "tell my mother I'll be late", ['mother', 'my mother']),
    ('city', 'I was born in Seville but I live in Madrid', ['madrid;seville']),
    ('animal', 'I have a dog and two cats', ['cats;dog', 'cat;dog']),
]

YESNO_EN = [
    ('yes, of course', 'YES'), ('no way', 'NO'), ("well, I don't know", 'UNKNOWN'),
    ('ok', 'YES'), ('alright, go ahead', 'YES'), ('better not', 'NO'), ('absolutely', 'YES'),
    ('no, thanks', 'NO'), ('agreed', 'YES'), ('nope', 'NO'), ('of course not', 'NO'),
    ("I'm not sure", 'UNKNOWN'), ('hmm, well, fine', 'YES'), ('never', 'NO'),
    ('good morning', 'UNKNOWN'), ('yes', 'YES'), ('no', 'NO'), ('sure', 'YES'),
    ('not now, maybe later', 'NO'), ('sounds good', 'YES'),
]

CASES = {'es': (EXTRACT_ES, YESNO_ES), 'en': (EXTRACT_EN, YESNO_EN)}


def normalize(answer):
    return ';'.join(sorted(v.strip().lower() for v in answer.split(';')))


def call(node, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
    if not future.done():
        return 'TIMEOUT'
    return future.result().result


def wait_for(node, client, name):
    if client.wait_for_service(timeout_sec=10.0):
        return True
    node.get_logger().error(f'Service {name} not available. Is simple_hri running?')
    return False


def test_extract(node, cases):
    client = node.create_client(Extract, '/extract_service')
    if not wait_for(node, client, '/extract_service'):
        return None
    print('\n/extract_service')
    ok = 0
    start = time.time()
    for interest, text, accepted in cases:
        result = call(node, client, Extract.Request(interest=interest, text=text))
        good = normalize(result) in accepted
        ok += good
        expected = '' if good else f'   (expected: {" or ".join(accepted)})'
        print(f"  {'✓' if good else '✗'} {interest:10} | {text:42} -> {result}{expected}")
    elapsed = (time.time() - start) / len(cases)
    print(f'  Extract: {ok}/{len(cases)} correct, {elapsed:.2f} s per request')
    return ok, len(cases)


def test_yesno(node, cases):
    client = node.create_client(YesNo, '/yesno_service')
    if not wait_for(node, client, '/yesno_service'):
        return None
    print('\n/yesno_service')
    ok = 0
    start = time.time()
    for text, expected in cases:
        result = call(node, client, YesNo.Request(text=text))
        good = result == expected
        ok += good
        note = '' if good else f'   (expected: {expected})'
        print(f"  {'✓' if good else '✗'} {text:25} -> {result}{note}")
    elapsed = (time.time() - start) / len(cases)
    print(f'  Yes/No: {ok}/{len(cases)} correct, {elapsed:.2f} s per request')
    return ok, len(cases)


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    language = next((a for a in argv if a in CASES), 'es')
    services = [a for a in argv if a in ('extract', 'yesno')] or ['extract', 'yesno']
    extract_cases, yesno_cases = CASES[language]
    node = rclpy.create_node('test_llm')
    try:
        if 'extract' in services:
            test_extract(node, extract_cases)
        if 'yesno' in services:
            test_yesno(node, yesno_cases)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
