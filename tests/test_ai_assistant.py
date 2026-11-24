"""Offline tests for assistant/apex_ai_assistant.py (no Hailo, Piper or Pi needed).

Run:  python -m pytest tests/test_ai_assistant.py   (or: python tests/test_ai_assistant.py)
"""
from __future__ import annotations

import http.client
import importlib.util
import json
import sys
import threading
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("apex_ai_assistant", ROOT / "assistant" / "apex_ai_assistant.py")
ai = importlib.util.module_from_spec(spec)
sys.modules["apex_ai_assistant"] = ai
sys.path.insert(0, str(ROOT / "assistant"))
spec.loader.exec_module(ai)

VISUAL = [
    "Ne görüyorsun?", "Kamerada ne var?", "Önümde ne var", "Bu nedir?", "Beni görüyor musun?",
    "ŞU AN NE GÖRÜYORSUN", "Elimde ne tutuyorum?", "Odayı anlat", "Bu ne renk?", "Kaç parmak gösteriyorum?",
    "Işıklar açık mı?", "Etrafına bak", "ne yapıyorum anlat", "el sallıyorum ne yapıyorum", "elimle kaç gösteriyorum",
    "ben geri gidicem 5 sn sonra ne görüdğünü tek tek anlat", "Etrafında insan var mı?", "Masada neler var?", "Görebiliyor musun beni?", "İleride kim var?",
]
NON_VISUAL = [
    "Bugünkü haberleri anlat", "Merhaba, nasılsın?", "12 çarpı 7 kaç eder?", "Sen kimsin?",
    "Bu ne demek?", "Görevin ne?", "Bana göre haklısın", "Görüşürüz", "Bakım zamanı geldi mi?",
    "Türkiye'nin başkenti neresi?", "Pil voltajın kaç?", "Bana bir fıkra anlat", "Bu konuda görüşün ne?",
    "Haberin var mı?", "Ne var ne yok?", "Sen kimsin, burada ne yapıyorsun?",
]
LIVE = [
    "Bugünkü haberleri anlat", "Son dakika ne var?", "Hava durumu nasıl?", "Dolar kaç lira?",
    "Güncel haberler neler", "Gündemde ne var", "Dün akşamki maç kaç kaç bitti?",
]
TRIAGE = [
    "Bileğimde ağrı hissediyorum", "Bacağım kırıldı galiba", "Kolum kanıyor", "Yardım edin!", "Sıkıştım, çıkamıyorum",
    "Nefes alamıyorum", "Başım dönüyor", "Ayağımı hareket ettiremiyorum", "Çok üşüyorum", "İMDAT", "Enkaz altındayım", "Bileğim şişti", "Yaralıyım", "Beni kurtarın",
]
NOT_TRIAGE = ["Masada şişe var mı?", "Sen kurtarma robotu musun?", "Bunun bana yararı ne?", "Merhaba", "Ne görüyorsun?", "Sen kimsin?", "Bugünkü haberleri anlat", "Kaç yaşındasın?", "Görevin ne?"]
CRITICAL = ["Nefes alamıyorum", "Enkaz altındayım", "Çok kan kaybediyorum", "Sıkıştım", "Bacağımı hissetmiyorum"]
NOT_LIVE = ["Bana haber ver", "Haberin var mı?", "Merhaba", "Ne görüyorsun?", "Altın oran nedir?"]


def test_visual_intent():
    for text in VISUAL:
        assert ai.wants_camera(text), text
    for text in NON_VISUAL:
        assert not ai.wants_camera(text), text


def test_live_intent():
    for text in LIVE:
        assert ai.wants_live_data(text), text
    for text in NOT_LIVE:
        assert not ai.wants_live_data(text), text


def test_triage_intent_and_plan():
    for text in TRIAGE:
        assert ai.wants_triage(text), text
    for text in NOT_TRIAGE:
        assert not ai.wants_triage(text), text
    for text in CRITICAL:
        assert ai.is_critical(text), text
    assert not ai.is_critical("Bileğimde ağrı hissediyorum")

    plan = ai.classify("Bileğimde ağrı hissediyorum")
    assert plan["intent"] == "triage" and plan["use_camera"] is False and plan["critical"] is False
    assert ai.medical_topics("Bileğimde ağrı hissediyorum") == ["pain", "wrist"]
    assert ai.medical_topics("Şişlik var ama kanama yok") == ["no bleeding", "swelling"]
    assert ai.classify("Nefes alamıyorum")["critical"] is True

    plan = ai.classify("ben geri gidicem 5 sn sonra ne görüdğünü tek tek anlat ne yapıyorum elimle kaç gösteriyorum")
    assert plan["intent"] == "visual" and plan["use_camera"] and plan["delay_s"] == 5.0
    assert plan["question_en"] == "What is the person doing right now? How many fingers is the person holding up?"
    assert ai.classify("Ne görüyorsun?")["question_en"].startswith("What do you see?")
    assert ai.classify("Etrafında insan var mı?")["question_en"] == "How many people do you see?"
    assert ai.classify("Şu tabelada ne yazıyor?")["question_en"] is None      # falls back to the translator
    assert ai.classify("90 saniye sonra bak ne görüyorsun")["delay_s"] == 20.0  # capped
    assert ai.classify("Bugünkü haberleri anlat")["intent"] == "live"
    assert ai.classify("Merhaba, nasılsın?")["intent"] == "chat" and not ai.classify("Merhaba")["use_camera"]
    assert ai.classify("Ne görüyorsun?", "off")["use_camera"] is False


def test_camera_modes():
    assert ai.parse_camera_mode({}) == "auto"
    assert ai.parse_camera_mode({"use_camera": True}) == "auto"   # legacy UI no longer forces a frame
    assert ai.parse_camera_mode({"use_camera": False}) == "off"
    assert ai.parse_camera_mode({"camera": "ON", "use_camera": False}) == "on"


def test_sentence_streaming():
    text = "I see a bed. The lamp is 5.06 meters away! 1. A bottle. Is it yours? Trailing part"
    buffer, got = "", []
    for char in text:            # token-by-token arrival
        buffer += char
        sentences, buffer = ai.split_sentences(buffer)
        got += sentences
    sentences, buffer = ai.split_sentences(buffer, final=True)
    got += sentences
    assert got == ["I see a bed.", "The lamp is 5.06 meters away!", "1. A bottle.", "Is it yours?", "Trailing part"], got
    assert buffer == ""
    assert ai.clean_en_sentence("1. **A bottle.**") == "A bottle."
    assert ai.clean_en_sentence("The lights are on.") == "The lights are turned on."
    assert ai.polish_tr_sentence("lambalar üzerindedir") == "Lambalar açık."
    assert ai.polish_tr_sentence("ışık açık .") == "Işık açık."
    assert ai.polish_tr_sentence("iyi günler") == "İyi günler."
    assert ai.polish_tr_sentence("şişe  ⁇ 'de ve ⁇  sağda") == "Şişe 'de ve sağda."


def user_text(prompt):
    return prompt[-1]["content"][-1]["text"]


def test_prompts():
    engine = ai.AssistantEngine(ai.parse_args([]))
    engine._camera_frame = lambda: (np.zeros((336, 336, 3), dtype=np.uint8), True)
    engine.history.extend([("user", "old question"), ("assistant", "I see a bed.")])

    # The word "camera" primed the model to answer "I see a camera".
    for text in (*ai.SYSTEM_PROMPTS.values(), ai.CRITICAL_NOTE, ai.INFRARED_NOTE, ai.BLIND_NOTE):
        assert "camera" not in text.lower(), text

    def build(message, english="translated"):
        plan = {**ai.classify(message), "message_tr": message}
        prompt, frames, _ = engine._prompt(english, plan)
        return prompt, frames, prompt[0]["content"][0]["text"], user_text(prompt)

    prompt, frames, system, user = build("Kamerada ne yazıyor?", "What does it say on camera?")
    assert len(frames) == 1 and prompt[-1]["content"][0] == {"type": "image"}
    assert user == "What does it say in front of you?"           # only the question, no instructions to echo
    assert "black and white" in system and len(prompt) == 2      # fresh look: no history

    _, _, _, user = build("ne yapıyorum, elimle kaç gösteriyorum")
    assert user == "What is the person doing right now? How many fingers is the person holding up?"

    prompt, frames, system, user = build("Bileğimde ağrı hissediyorum", "I feel pain in my wrist.")
    assert frames == [] and user == "I feel pain in my wrist."   # triage is text-only
    assert "first-aid" in system and "pain, wrist" in system and "life-threatening" not in system
    assert len(prompt) == 4                                      # history kept for follow-ups
    assert "alerting the rescue operator" in build("Nefes alamıyorum")[2]

    prompt, frames, system, user = build("Bugünkü haberleri anlat", "Tell me today's news.")
    assert frames == [] and "without internet" in system and user == "Tell me today's news."

    prompt, frames, system, user = build("Merhaba, nasılsın?", "Hello, how are you?")
    assert frames == [] and user == "Hello, how are you?"

    plan = {**ai.classify("Ne görüyorsun?", "off"), "message_tr": "Ne görüyorsun?"}
    prompt, frames, _ = engine._prompt("x", plan)
    assert frames == [] and "cannot see" in prompt[0]["content"][0]["text"]


def test_loop_breaker():
    kept = ["The person is very close, and they are wearing a white shirt."]
    assert ai.looks_repeated("The person is very close, and they are wearing a white shirt.", kept)
    assert ai.looks_repeated("The person is very close to you and near the wall.", kept)          # same opening
    assert ai.looks_repeated("They are close to the person in the center, close to the person in the center.", [])
    assert not ai.looks_repeated("You are raising your right hand.", kept)
    assert not ai.looks_repeated("Is it swollen?", ["Is it bleeding?"])

    class Generation:
        def __init__(self, text): self.text = text
        def __enter__(self): return iter(word + " " for word in self.text.split(" "))
        def __exit__(self, *exc): return False

    class Vlm:
        def generate(self, prompt, frames, **kwargs):
            self.frames = frames
            return Generation(self.text)
        def clear_context(self): pass

    class Translator:
        def translate(self, text, pair): return text

    engine = ai.AssistantEngine(ai.parse_args([]))
    engine.vlm, engine.translator = Vlm(), Translator()
    engine._camera_frame = lambda: (np.zeros((336, 336, 3), dtype=np.uint8), False)
    engine.vlm.text = ("You are waving your hand. You are showing five fingers. You are waving your hand. "
                       "You are waving your hand. You are waving your hand.")
    answer = list(engine.generate("ne yapıyorum", ai.classify("ne yapıyorum")))
    assert answer == ["You are waving your hand.", "You are showing five fingers."], answer
    assert engine.trace[-1]["stopped_early"] is True and len(engine.vlm.frames) == 1
    assert not engine.inference_lock.locked()
    engine.vlm.text = "One thing. Second thing here. Third thing here. Fourth thing here. Fifth thing here."
    assert len(list(engine.generate("Merhaba", ai.classify("Merhaba")))) == 3           # chat is capped at 3 sentences


def test_voices():
    for name in ai.VOICES:
        chain = ai.voice_filter(name, 22_050)
        assert "{" not in chain and chain
    assert ai.voice_filter("bogus", 22_050) == ai.voice_filter(ai.DEFAULT_VOICE, 22_050)
    assert ai.pick_voice("dogal") == "dogal" and ai.pick_voice(None) == ai.DEFAULT_VOICE and ai.pick_voice("x", "robot") == "robot"


def test_scene_detector():
    import apex_scene as scene
    from PIL import Image

    model = ROOT / "assistant" / "models" / "yolox_tiny.onnx"
    try:
        detector = scene.YoloxDetector(model)
    except Exception as exc:            # onnxruntime or model not on this machine
        print("   (detector test skipped:", type(exc).__name__, ")")
        return
    for name, expect_ir in (("person_960x540.jpg", False), ("person_ir_960x540.jpg", True)):
        image, infrared = scene.prepare_frame(Image.open(ROOT / "tests" / "data" / name))
        assert infrared is expect_ir, name
        found = detector.detect(image)
        people = [item for item in found if item["label"] == "person"]
        assert len(people) == 1, (name, found)
        x1, y1, x2, y2 = people[0]["box"]
        assert 0.2 < x1 < 0.45 and 0.55 < x2 < 0.8 and people[0]["pos"] == "center", people[0]
        assert scene.summarize(found).startswith("person (center")
        assert scene.summarize(found, turkish=True).startswith("insan (ortada")
    assert scene.summarize([]) == "no known object detected"
    blank = detector.detect(Image.new("RGB", (640, 360), (20, 20, 20)))
    assert blank == []
    lying = scene.describe_box([0.1, 0.5, 0.8, 0.8], "person", 16 / 9)
    assert lying["posture"] == "lying down"
    assert scene.describe_box([0.0, 0.0, 1.0, 1.0], "person", 16 / 9)["posture"] == "upright"   # close-up: unknown


def test_speech_level():
    rate = 22_050
    t = np.arange(rate * 2) / rate
    envelope = (np.sin(2 * np.pi * 3 * t) > 0) * (0.4 + 0.6 * np.abs(np.sin(2 * np.pi * 1.3 * t)))
    rng = np.random.default_rng(1)
    for scale in (0.02, 0.2, 0.95):                      # quiet, normal, hot input
        voice = envelope * (np.sin(2 * np.pi * 140 * t) + 0.5 * np.sin(2 * np.pi * 900 * t) + 0.3 * rng.standard_normal(t.size))
        pcm = np.asarray(voice / np.abs(voice).max() * scale * 32767, dtype="<i2")
        out = ai.normalize_speech(pcm, rate, -17.0, -1.0)
        rms, peak = ai.pcm_stats(out, rate)
        assert peak <= -0.99, (scale, peak)                # never above the ceiling -> no clipping
        assert -19.5 <= rms <= -16.0, (scale, rms)         # consistent loudness regardless of input
    assert ai.normalize_speech(np.zeros(100, dtype="<i2"), rate).max() == 0


class FakeEngine(ai.AssistantEngine):
    """Real HTTP handler and routing, fake VLM/translator/audio."""

    def __init__(self):
        super().__init__(ai.parse_args([]))
        self.vlm = object()
        self.calls = []
        self.spoken = []

    def generate(self, message, plan):
        self.calls.append((message, plan["intent"], plan["use_camera"]))
        self.last_inference_s, self.first_sentence_s = 1.5, 0.5
        yield "Bir insan görüyorum." if plan["use_camera"] else "Merhaba."
        yield "İkinci cümle."

    def speak_async(self, text, voice=None):
        self.spoken.append(("say", text, voice))
        return True

    def begin_speech(self, voice=None):
        engine = self

        class Session:
            def say(self, text): engine.spoken.append(text)
            def finish(self): engine.spoken.append(None)
        return Session()


@contextmanager
def serve():
    engine = FakeEngine()
    server = ThreadingHTTPServer(("127.0.0.1", 0), ai.AssistantHandler)
    server.engine, server.index_html = engine, b"<html></html>"
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        yield engine, server.server_address[1]
    finally:
        server.shutdown()
        server.server_close()


def chat(port, body):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    conn.request("POST", "/api/chat", json.dumps(body), {"Content-Type": "application/json"})
    response = conn.getresponse()
    events = [json.loads(line) for line in response.read().decode().splitlines() if line]
    conn.close()
    return response.status, events


def test_api_routing():
    with serve() as (engine, port):
        status, events = chat(port, {"message": "Ne görüyorsun?", "use_camera": True, "speak": True, "voice": "dogal"})
        assert status == 200 and engine.calls[-1] == ("Ne görüyorsun?", "visual", True)
        assert events[0]["intent"] == "visual" and events[-1]["camera_used"] is True
        assert events[-1]["text"] == "Bir insan görüyorum. İkinci cümle."
        assert engine.spoken == ["Bir insan görüyorum.", "İkinci cümle.", None]   # sentence-by-sentence TTS

        status, events = chat(port, {"message": "Bileğimde ağrı hissediyorum"})
        assert engine.calls[-1] == ("Bileğimde ağrı hissediyorum", "triage", False)
        assert events[0]["critical"] is False and events[0]["text"].startswith("DURUM DEĞERLENDİRME")
        status, events = chat(port, {"message": "Nefes alamıyorum"})
        assert events[0]["critical"] is True and events[-1]["critical"] is True

        status, events = chat(port, {"message": "Bugünkü haberleri anlat", "use_camera": True})
        assert engine.calls[-1] == ("Bugünkü haberleri anlat", "live", False) and events[-1]["camera_used"] is False

        status, events = chat(port, {"message": "Merhaba, nasılsın?", "use_camera": True})
        assert engine.calls[-1] == ("Merhaba, nasılsın?", "chat", False)

        status, events = chat(port, {"message": "Ne görüyorsun?", "camera": "off"})
        assert engine.calls[-1][2] is False

        status, _ = chat(port, {"message": "  "})
        assert status == 400

        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=10)

        def call(method, path, body=None):
            conn.request(method, path, None if body is None else json.dumps(body), {"Content-Type": "application/json"})
            response = conn.getresponse()
            return response.status, json.loads(response.read())

        status, health = call("GET", "/api/health")
        for key in ("ok", "audio_available", "voice_ready", "red_alert_ready", "translation_ready", "busy",
                    "speaking", "last_error", "tts_error", "power_voltage_v", "power_flags",
                    "undervoltage_now", "undervoltage_seen", "voices", "default_voice", "scene"):
            assert key in health, key
        assert call("GET", "/api/scene") == (200, {"ok": False, "enabled": False})
        assert call("GET", "/api/trace")[0] == 200
        assert call("POST", "/api/reset") == (200, {"ok": True})
        assert call("POST", "/api/sound", {"name": "bogus"})[0] == 400
        assert call("POST", "/api/say", {"text": "Deneme.", "voice": "telsiz"})[0] == 202
        assert engine.spoken[-1] == ("say", "Deneme.", "telsiz")
        assert call("POST", "/api/say", {"text": ""})[0] == 400
        conn.close()


if __name__ == "__main__":
    for name, function in list(globals().items()):
        if name.startswith("test_") and callable(function):
            function()
            print("ok ", name)
