#!/usr/bin/env python3
"""APEX rescue assistant for the META-1 hexapod (Hailo-10H, fully offline).

The camera remains owned by ``apex-camera-test.service``.  This service only
reads a current JPEG through HTTP, which prevents CSI device contention and
keeps the 1080p browser stream independent from VLM inference.

How a request is handled (v2):

1. ``apex_scene.SceneWatcher`` runs a YOLOX person/object detector about once
   per second, all the time.  ``/api/scene`` feeds the operator overlay and
   the result is given to the VLM as grounding.
2. The Turkish message is classified before anything reaches the VLM:
   *triage* (pain, injury, trapped, help), *visual*, *live* (needs internet)
   or *chat*.  Triage and visual requests get a fresh night-vision frame.
3. The English VLM output is split into sentences while it is generated;
   each sentence is translated to Turkish and streamed immediately.
4. With speech enabled each sentence goes straight into a queued TTS
   pipeline (persistent Piper voice -> voice character filter -> loudness
   normaliser -> one ``aplay`` stream).

Nothing is answered from canned text; ``/api/trace`` shows the exact prompt
and raw model output of the last requests for tuning.

This service never talks to the motor/servo/ESP side.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
import urllib.request
import wave
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterator

import numpy as np

# Heavy, Pi-only dependencies (hailo_platform, ctranslate2, sentencepiece,
# Pillow, piper) are imported lazily where they are used so that the pure
# logic in this file can be unit-tested on any machine.

VERSION = "2.1"

# Lessons from the on-robot trace (Qwen2-VL-2B is small):
#  * the word "camera" in a prompt is parroted back ("I see a camera");
#  * long instructions in the *user* turn are echoed instead of followed,
#    so every instruction lives in a short per-intent system prompt and the
#    user turn carries only the question;
#  * an attached image hijacks first-aid answers, so triage is text-only.
SYSTEM_PROMPTS = {
    "visual": (
        "You are APEX, a rescue robot. The image shows what is in front of you "
        "right now, and the person in it is the one talking to you. Answer "
        "their question in two or three short sentences. Say what you really "
        "see: the person, what they are doing, their hands and gestures, and "
        "the objects around. Call the person 'you'. Never repeat a sentence."
    ),
    "triage": (
        "You are APEX, a rescue robot talking with an injured earthquake "
        "survivor. Reply with exactly three short sentences: one calming "
        "sentence, one clear first-aid instruction for their problem that "
        "they can do alone right now, and one question about their condition. "
        "Do not name medicines. Never repeat a sentence."
    ),
    "live": (
        "You are APEX, an offline rescue robot without internet. You cannot "
        "know news, weather, prices or sports scores. Say this honestly in "
        "one short sentence, then offer your help in one short sentence."
    ),
    "chat": (
        "You are APEX, a friendly six-legged rescue robot that helps "
        "earthquake survivors and talks with its operator. You can see in the "
        "dark, detect people and objects, speak and sound alarms. You have no "
        "internet. Answer in one to three short, simple sentences. Never "
        "repeat a sentence."
    ),
}
CRITICAL_NOTE = (
    " This is life-threatening: tell them to stay still and that you are "
    "alerting the rescue operator right now."
)
INFRARED_NOTE = " The image is black and white, so do not mention colors."
BLIND_NOTE = " Your eyes are switched off right now, so say honestly that you cannot see."
MAX_SENTENCES = {"visual": 4, "triage": 3, "live": 2, "chat": 3}

# Turkish -> English topic words for triage.  The offline translator mangles
# medical vocabulary ("şişlik" became "skid"), so the key facts are handed to
# the model directly.
_MEDICAL_WORDS = tuple((re.compile(pattern), word) for pattern, word in (
    (r"\bağr[ıi]\w*|\bac[ıi]yor\w*|\bsanc[ıi]\w*|\bs[ıi]zl[ıi]\w*", "pain"),
    (r"\bkan[ıi]yor\w*|\bkanama\s+(?:var|durmuyor)\w*|\bkan\s+(?:var|geliyor|ak[ıi]yor)\w*", "bleeding"),
    (r"\bkanama\s+yok\b", "no bleeding"),
    (r"\bşiş(?:ti|lik|miş|iyor|kin)\w*", "swelling"), (r"\bmorar\w*", "bruise"),
    (r"\bk[ıi]r[ıi]k\w*|\bk[ıi]r[ıi]ld[ıi]\w*", "possible fracture"), (r"\bburk\w*", "sprain"),
    (r"\byan[ıi]k\w*", "burn"), (r"\bkesik\w*|\byara\w*", "wound"), (r"\buyuş\w*|\bhissetmiyorum\b", "numbness"),
    (r"\bnefes\w*|\bsoluk\w*|\bboğul\w*", "trouble breathing"), (r"\bbaş[ıi]m\s+dön\w*", "dizziness"),
    (r"\bbay[ıi]l\w*|\bbilin[cç]\w*", "fainting"), (r"\bs[ıi]k[ıi]ş\w*|\benkaz\w*|\balt[ıi]nda\s+kald\w*", "trapped under debris"),
    (r"\bhareket\s+(?:edem|ettirem)\w*", "cannot move it"), (r"\büşü\w*", "feeling cold"), (r"\bsusad\w*", "thirsty"),
    (r"\bkorku\w*", "afraid"), (r"\bbile[ğk]\w*", "wrist"), (r"\bayak\w*|\baya[ğg]\w*", "foot"), (r"\bbaca[ğk]\w*", "leg"),
    (r"\bkol(?:um|umu|umda|u)\b", "arm"), (r"\bel(?:im|imi|imde|lerim)\w*", "hand"), (r"\bparma\w*", "finger"), (r"\bbaş[ıi]m\w*|\bkafa\w*", "head"),
    (r"\bgöğ[sü]\w*", "chest"), (r"\bs[ıi]rt\w*", "back"), (r"\bboy(?:un|nu)\w*", "neck"), (r"\bdiz(?:im|imi|imde)\w*", "knee"),
    (r"\bomuz\w*", "shoulder"), (r"\bkar(?:ın|nım)\w*|\bmide\w*", "stomach"), (r"\bgöz(?:üm|ümü|ümde|lerim)\w*", "eye"),
))


def medical_topics(message: str) -> list[str]:
    folded = fold_tr(message)
    found: list[str] = []
    for pattern, word in _MEDICAL_WORDS:
        if word not in found and pattern.search(folded):
            found.append(word)
    return found


# Common visual questions are asked in fixed, clean English instead of going
# through the translator, which garbles long spoken-style Turkish sentences.
_VISUAL_SLOTS = tuple((re.compile(pattern), question) for pattern, question in (
    (r"\bne\s+yap[ıi]yorum\b|\bne\s+yap[ıi]yor\b|\bsall[ıi]yor\w*|\bhareketimi\w*", "What is the person doing right now?"),
    (r"\bparma\w*|\bkaç\s+(?:tane\s+)?gösteri\w*|\belimle\s+kaç\b", "How many fingers is the person holding up?"),
    (r"\belimde\w*|\bne\s+tutuyor\w*", "What is the person holding in their hand?"),
    (r"\bne\s+giy\w*|\büzerimde\w*|\büstümde\w*", "What is the person wearing?"),
    (r"\bkaç\s+kişi\w*|\binsan\s+var\b|\bkim(?:se)?\s+var\b", "How many people do you see?"),
))
_DELAY = re.compile(r"\b(\d{1,2})\s*(?:sn|saniye)\w*")


def visual_question(message: str) -> str | None:
    """Clean English question for a recognised visual request, else None."""
    folded = fold_tr(message)
    parts = [question for pattern, question in _VISUAL_SLOTS if pattern.search(folded)]
    if not parts and _SIMPLE_LOOK.search(folded):
        parts = ["What do you see? Describe the person and the objects."]
    return " ".join(parts[:3]) or None


def requested_delay(message: str) -> float:
    """Seconds to wait before looking ("5 sn sonra bak"), capped at 20."""
    folded = fold_tr(message)
    match = _DELAY.search(folded)
    if not match or not re.search(r"\bsonra\b|\bbekle\w*|\bsay\b|\biçinde\b", folded):
        return 0.0
    return float(min(int(match.group(1)), 20))


SPECIAL_TOKENS = ("<|im_end|>", "<|endoftext|>", "<|eot_id|>")

CAMERA_MODES = ("auto", "on", "off")


# --------------------------------------------------------------------------
# Intent routing (works on the original Turkish text, before translation)
# --------------------------------------------------------------------------

def fold_tr(text: str) -> str:
    """Turkish-aware lower-casing (İ->i, I->ı) with collapsed whitespace."""
    text = text.replace("İ", "i").replace("I", "ı").lower()
    return re.sub(r"\s+", " ", text).strip()


_VISUAL_PATTERNS = tuple(re.compile(pattern) for pattern in (
    # görmek in any form (typos included), but not görev / göre / görüş
    r"\bgör(?!ev|e\b|eli|eceli|üş)\w+",
    r"\bne\s+yap[ıi]yorum\b|\bne\s+yap[ıi]yor\b",
    r"\belim(?:le|i|de|deki)\w*|\bparma\w*|\bgösteriyor\w*|\bsall[ıi]yor\w*",
    r"\binsan\s+var\b|\bkimse\s+var\b|\bbana\s+bak\w*|\bbeni\s+(?:anlat|tarif|izle)\w*",
    # kamera as a place/direction: kamerada, kameraya, kameranda ...
    r"\bkamera(?:da|ya|nda|na|dan|ndan|yla)\w*",
    # bakmak (imperative / request), not "bakım" or "bakan"
    r"\bbak\b|\bbaksana\b|\bbakar\s+m[ıi]s[ıi]n\w*|\bbakabilir\w*|\bbakıp\b",
    # deictic places relative to the robot or the user
    r"\b(?:önümde|önünde|karşımda|karşında|arkamda|yanımda|elimde|üzerimde|üstümde)\w*",
    r"\b(?:çevremde|çevrende)\w*|\betraf[ıi]\w*",
    # "bu ne / bu nedir / şu kim" but not "bu ne demek / bu ne zaman"
    r"\b(?:bu|şu|bunlar|şunlar)\s+(?:nedir|neler|ne|kimdir|kim)\b"
    r"(?!\s+(?:demek|zaman|kadar|için|işe|anlama))",
    r"\bne\s+renk\w*|\brengi\w*|\brenkte\b",
    r"\bkaç\s+(?:parmak|kişi)\w*",
    r"\b(?:odada|odayı|odanın|masada|masadaki|masanın|duvarda)\w*",
    r"\b(?:betimle|sahne|ortamı|manzara)\w*",
    r"\b(?:fotoğraf|resimde|karede)\w*",
    r"\bne\s+giy\w*|\bne\s+tutuyor\w*|\bne\s+yazıyor\b",
    r"\b(?:yazıyı|yazanı|etiketi|ekranı)\s+oku\w*",
    r"\bkim\s+var\b|\bburada\s+(?:ne|neler|kim|kaç\s+kişi)\s+var\b|\bburası\s+neresi\b",
    r"\bışık\w*\s+(?:açık|kapalı|yanıyor)\w*",
    r"\bbeni\s+tan[ıi]\w*",
))

_LIVE_PATTERNS = tuple(re.compile(pattern) for pattern in (
    r"\bhaberler\w*|\bhaberleri\w*",
    r"\b(?:güncel|son|bugünkü|bugünün|yeni|dünkü)\s+haber\w*",
    r"\bson\s+dakika\b|\bgündem\w*",
    r"\bhava\s+(?:durumu|nasıl|kaç\s+derece)\w*|\byağmur\s+yağacak\w*",
    r"\b(?:dolar|euro|sterlin|bitcoin|borsa|altın)\b.*\b(?:kaç|ne\s+kadar|fiyat\w*|kur\w*|bugün)\b",
    r"\b(?:dolar|euro)\s+kuru\b",
    r"\bmaç\w*\s+(?:skor\w*|sonuc\w*|sonuç\w*|kaç\s+kaç)|\bkaç\s+kaç\b",
    r"\binternette\w*\s+(?:ara|bak)\w*",
))


_MEDICAL_PATTERNS = tuple(re.compile(pattern) for pattern in (
    r"\bağr[ıi]\w*|\bac[ıi]yor\w*|\bac[ıi]\s+(?:var|çek)\w*|\bsanc[ıi]\w*|\bs[ıi]zl[ıi]\w*",
    r"\byaral[ıi]\w*|\byara(?:m|n|s[ıi]|lar\w*)?\b|\bkan[ıi]yor\w*|\bkanama\w*|\bkan\s+(?:var|geliyor|ak[ıi]yor)\w*",
    r"\bk[ıi]r[ıi]k\w*|\bk[ıi]r[ıi]ld[ıi]\w*|\bburk\w*|\bçıkık\w*|\bşiş(?:ti|lik|miş|iyor|kin)\w*|\bmorar\w*|\byan[ıi]k\w*|\bkesik\w*|\bezil\w*",
    r"\bs[ıi]k[ıi]ş\w*|\bkurtar(?:[ıi]n|sana)?\b|\bbeni\s+kurtar\w*|\bimdat\b|\byard[ıi]m\s+(?:et|edin|edebilir|laz[ıi]m|gerek|istiyorum)\w*|\byard[ıi]m[ıi]na\b",
    r"\bnefes\w*|\bsoluk\w*|\bboğul\w*|\bbay[ıi]l\w*|\bbaş[ıi]m\s+dön\w*|\bmidem\s+bulan\w*|\bbilin[cç]\w*",
    r"\bhareket\s+(?:edem|edebil|ettirem|etmiyor)\w*|\bk[ıi]p[ıi]rda\w*|\buyuş\w*|\bhisset(?:mi|em)\w*|\büşü\w*|\bsusad\w*|\bkorku\w*",
    r"\benkaz\w*\s+alt\w*|\balt[ıi]nda\s+kald\w*|\biyi\s+değilim\b|\bkötüyüm\b|\bölüyor\w*|\bgöğs\w*|\bkalb\w*",
))

_CRITICAL_PATTERNS = tuple(re.compile(pattern) for pattern in (
    r"\bnefes\s+al(?:am|amı)\w*|\bboğul\w*|\bçok\s+kan\w*|\bkan\w*\s+durmuyor\b|\bbilin[cç]\w*",
    r"\bgöğs\w*\s+(?:ağr|s[ıi]k[ıi]ş)\w*|\bkalb\w*|\bbay[ıi]l\w*|\bs[ıi]k[ıi]şt[ıi]m\b|\benkaz\w*\s+alt\w*",
    r"\balt[ıi]nda\s+kald\w*|\bhareket\s+(?:edem|ettirem)\w*|\bölüyor\w*|\bhissetmiyorum\b",
))

_SIMPLE_LOOK = re.compile(
    r"\b(?:ne|neler)\s+gör\w+|\bgör\w+\s+(?:anlat|söyle)\w*|\betraf\w*\s+(?:anlat|bak|ne\s+var)\w*"
    r"|\b(?:önünde|karşında|kamerada)\s+ne(?:ler)?\s+var\b"
)


def wants_camera(message: str) -> bool:
    """True when the Turkish message is a question about what the robot sees."""
    folded = fold_tr(message)
    return any(pattern.search(folded) for pattern in _VISUAL_PATTERNS)


def wants_live_data(message: str) -> bool:
    """True for requests that need a live source the robot does not have."""
    folded = fold_tr(message)
    return any(pattern.search(folded) for pattern in _LIVE_PATTERNS)


def wants_triage(message: str) -> bool:
    """True when a person reports pain, injury, entrapment or asks for help."""
    folded = fold_tr(message)
    return any(pattern.search(folded) for pattern in _MEDICAL_PATTERNS)


def is_critical(message: str) -> bool:
    folded = fold_tr(message)
    return any(pattern.search(folded) for pattern in _CRITICAL_PATTERNS)


def classify(message: str, camera_mode: str = "auto") -> dict[str, Any]:
    """Decide how a Turkish message is handled.

    intent: "triage" (injury / help), "visual", "live" (needs internet) or "chat".
    A fresh frame is attached only for visual questions.
    """
    if wants_triage(message):
        intent = "triage"
    elif wants_camera(message):
        intent = "visual"
    elif wants_live_data(message):
        intent = "live"
    else:
        intent = "chat"
    # Triage is text-only: an attached image makes the small model describe
    # the picture instead of giving first aid.
    use_camera = intent == "visual" and camera_mode != "off"
    return {
        "intent": intent,
        "use_camera": use_camera,
        "critical": intent == "triage" and is_critical(message),
        "question_en": visual_question(message) if intent == "visual" else None,
        "delay_s": requested_delay(message) if use_camera else 0.0,
    }


def parse_camera_mode(data: dict[str, Any]) -> str:
    """Read the camera policy from a chat request.

    ``camera`` may be "auto" (default), "on" or "off".  The legacy boolean
    ``use_camera`` is still accepted: ``true`` now means *auto* (attach a
    frame only for visual questions) and ``false`` means *off*.
    """
    mode = data.get("camera")
    if isinstance(mode, str) and mode.lower() in CAMERA_MODES:
        return mode.lower()
    if "use_camera" in data:
        return "auto" if bool(data.get("use_camera")) else "off"
    return "auto"


def resolve_camera(mode: str, message: str) -> bool:
    if mode == "on":
        return True
    if mode == "off":
        return False
    return wants_camera(message)


# --------------------------------------------------------------------------
# Sentence streaming and translation hygiene
# --------------------------------------------------------------------------

_SENTENCE_END = re.compile(r"""(?:[.!?…]+["')\]]*(?=\s|$)|\n+)""")
_LIST_MARKER = re.compile(r"^\s*(?:[-*•]+|\d+[.)])\s*")
_ABBREVIATION = re.compile(r"\b(?:e\.g|i\.e|mr|mrs|ms|dr|vs|etc|approx|no)\.$", re.IGNORECASE)


def split_sentences(buffer: str, final: bool = False) -> tuple[list[str], str]:
    """Cut complete sentences off the front of a growing text buffer.

    Returns ``(sentences, rest)``.  A terminator at the very end of the buffer
    is not trusted until more text arrives (it may be "5." of "5.06"), unless
    ``final`` is set, in which case everything left is flushed.
    """
    sentences: list[str] = []
    start = 0
    for match in _SENTENCE_END.finditer(buffer):
        if match.end() == len(buffer) and not final:
            break
        candidate = buffer[start:match.end()].strip()
        if not candidate:
            start = match.end()
            continue
        if not match.group().startswith("\n") and (
            _LIST_MARKER.fullmatch(candidate) or _ABBREVIATION.search(candidate)
        ):
            continue  # "1." or "e.g." - keep accumulating
        sentences.append(candidate)
        start = match.end()
    rest = buffer[start:]
    if final:
        if rest.strip():
            sentences.append(rest.strip())
        rest = ""
    return sentences, rest


_EN_SWITCH = re.compile(
    r"\b(lights?|lamps?|tv|television|screen|monitor|computer|fan|camera)\s+(is|are)\s+(on|off)\b",
    re.IGNORECASE,
)


def clean_en_sentence(sentence: str) -> str:
    """Strip markdown/list noise and phrases the EN->TR model mistranslates."""
    sentence = _LIST_MARKER.sub("", sentence)
    sentence = re.sub(r"[*_#`>|]+", " ", sentence)
    sentence = re.sub(r"^(?:APEX|Assistant|Answer|Robot)\s*:\s*", "", sentence, flags=re.IGNORECASE)
    # "the lights are on" is translated literally ("üzerindedir"); the
    # participle form survives translation as "açık / kapalı".
    sentence = _EN_SWITCH.sub(lambda m: f"{m.group(1)} {m.group(2)} turned {m.group(3)}", sentence)
    return re.sub(r"\s+", " ", sentence).strip()


_TR_FIXES = tuple((re.compile(pattern), replacement) for pattern, replacement in (
    (r"\b(ışık|ışıklar|lamba|lambalar|ekran|televizyon)\s+üzerinde(?:dir|ler)?(?=[\s.,!?]|$)", r"\1 açık"),
    (r"\s*⁇\s*", " "),
    (r"\barama-ve-rescue\b|\bsearch-and-rescue\b|\brescue\b", "kurtarma"),
    (r"\s+([.,!?;:])", r"\1"),
    (r"([.!?])\1+", r"\1"),
))


def polish_tr_sentence(sentence: str) -> str:
    """Light, rule-based cleanup of a translated Turkish sentence."""
    for pattern, replacement in _TR_FIXES:
        sentence = pattern.sub(replacement, sentence)
    sentence = re.sub(r"\s+", " ", sentence).strip()
    if not sentence:
        return ""
    first = sentence[0]
    upper = "İ" if first == "i" else "I" if first == "ı" else first.upper()
    sentence = upper + sentence[1:]
    if sentence[-1] not in ".!?…":
        sentence += "."
    return sentence


def looks_repeated(sentence: str, kept: list[str]) -> bool:
    """True when a sentence (nearly) repeats an earlier one of the same answer."""
    words = re.findall(r"\w+", sentence.lower())
    if len(words) < 3:
        return False
    for earlier in kept:
        other = re.findall(r"\w+", earlier.lower())
        if words == other or (len(words) >= 5 and words[:5] == other[:5]):
            return True
        common = len(set(words) & set(other))
        if len(words) >= 6 and common / len(set(words)) > 0.8 and common / max(len(set(other)), 1) > 0.8:
            return True
    # a clause repeated inside one sentence ("... close to the person ... close to the person")
    for size in (4, 5):
        grams = [tuple(words[i:i + size]) for i in range(len(words) - size + 1)]
        if len(grams) != len(set(grams)):
            return True
    return False


def speech_text(text: str) -> str:
    text = re.sub(r"```.*?```", " kod bloğu ", text, flags=re.DOTALL)
    text = re.sub(r"https?://\S+", " bağlantı ", text)
    text = re.sub(r"[*_#`>|]", " ", text)
    return re.sub(r"\s+", " ", text).strip()[:800]


# --------------------------------------------------------------------------
# Speech level: measured gain + look-ahead limiter (no clipping)
# --------------------------------------------------------------------------

def pcm_stats(pcm: np.ndarray, rate: int) -> tuple[float, float]:
    """Return (gated RMS dBFS, peak dBFS) of int16 mono PCM.

    The RMS only counts 50 ms blocks within 30 dB of the loudest block, so
    pauses between words do not drag the measured speech level down.
    """
    if pcm.size == 0:
        return -120.0, -120.0
    x = pcm.astype(np.float32) / 32768.0
    peak = float(np.max(np.abs(x)))
    block = max(1, int(rate * 0.05))
    count = len(x) // block
    if count == 0:
        rms = float(np.sqrt(np.mean(x ** 2)))
    else:
        blocks = np.sqrt(np.mean(x[: count * block].reshape(count, block) ** 2, axis=1))
        active = blocks[blocks > max(float(blocks.max()) * 10 ** (-30 / 20), 1e-5)]
        rms = float(np.sqrt(np.mean(active ** 2))) if active.size else 0.0
    to_db = lambda value: 20 * math.log10(value) if value > 1e-6 else -120.0  # noqa: E731
    return to_db(rms), to_db(peak)


def _limit(x: np.ndarray, rate: int, ceiling: float) -> np.ndarray:
    """Look-ahead peak limiter: smooth gain reduction, never above ceiling."""
    magnitude = np.abs(x)
    if float(magnitude.max(initial=0.0)) <= ceiling:
        return x
    half = max(1, int(rate * 0.0015))
    width = 2 * half + 1
    gain = np.minimum(1.0, ceiling / np.maximum(magnitude, 1e-9)).astype(np.float32)
    padded = np.pad(gain, half, constant_values=1.0)
    lowest = np.lib.stride_tricks.sliding_window_view(padded, width).min(axis=1)
    padded = np.pad(lowest, half, mode="edge")
    smooth = np.convolve(padded, np.full(width, 1.0 / width, dtype=np.float32), mode="valid")
    # mean(min(...)) over the same window is always <= the instantaneous gain,
    # so the result cannot overshoot; the clip is only a numeric safety net.
    return np.clip(x * smooth, -ceiling, ceiling)


def normalize_speech(
    pcm: np.ndarray, rate: int, target_dbfs: float = -17.0, peak_dbfs: float = -1.0
) -> np.ndarray:
    """Bring speech to a fixed loudness and cap true sample peaks.

    A gated RMS of about -17 dBFS corresponds to roughly -16/-15 LUFS for
    speech; peaks are limited to ``peak_dbfs`` instead of being clipped.
    """
    if pcm.size == 0:
        return pcm
    level, _ = pcm_stats(pcm, rate)
    if level <= -100.0:
        return pcm
    gain_db = float(np.clip(target_dbfs - level, -20.0, 30.0))
    x = pcm.astype(np.float32) / 32768.0 * (10 ** (gain_db / 20))
    x = _limit(x, rate, 10 ** (peak_dbfs / 20))
    return np.asarray(np.round(x * 32767.0), dtype="<i2")


# Selectable voice characters.  The final loudness is always set by
# ``normalize_speech``; the chains only shape the timbre.
VOICES: dict[str, dict[str, Any]] = {
    "dogal": {
        "label": "DOĞAL · net insan sesi",
        "filter": "highpass=f=70,acompressor=threshold=0.12:ratio=3:attack=5:release=90:makeup=1.5",
    },
    "tok": {
        "label": "TOK · kalın ve net",
        "filter": (
            "asetrate={rate}*0.93,aresample={rate},atempo=1.07527,"
            "highpass=f=65,bass=g=5:f=150:w=0.7,equalizer=f=3000:t=q:w=1.0:g=3,"
            "acompressor=threshold=0.1:ratio=4:attack=4:release=90:makeup=2"
        ),
    },
    "robot": {
        "label": "ROBOT · metalik",
        "filter": (
            "asetrate={rate}*0.88,aresample={rate},atempo=1.13636,"
            "highpass=f=70,bass=g=4:f=160:w=0.7,equalizer=f=3200:t=q:w=1.0:g=3,"
            "aecho=0.9:0.55:9|17:0.30|0.20,"
            "flanger=delay=2:depth=1:regen=0:width=35:speed=0.3,lowpass=f=7000,"
            "acompressor=threshold=0.1:ratio=4:attack=4:release=90:makeup=2"
        ),
    },
    "telsiz": {
        "label": "TELSİZ · dar bant, uzaktan duyulur",
        "filter": (
            "highpass=f=320,lowpass=f=3400,equalizer=f=1800:t=q:w=0.8:g=4,"
            "acompressor=threshold=0.06:ratio=8:attack=2:release=60:makeup=3"
        ),
    },
}
DEFAULT_VOICE = "tok"


def voice_filter(voice: str, rate: int) -> str:
    """FFmpeg filter chain of a voice character (unknown names -> default)."""
    preset = VOICES.get(voice) or VOICES[DEFAULT_VOICE]
    return preset["filter"].format(rate=rate)


def pick_voice(value: Any, default: str = DEFAULT_VOICE) -> str:
    return value if isinstance(value, str) and value in VOICES else default


# --------------------------------------------------------------------------
# Offline translation
# --------------------------------------------------------------------------

class OfflineTranslator:
    """Small persistent CTranslate2 pair; no PyTorch or cloud service required."""

    def __init__(self, root: Path) -> None:
        import ctranslate2
        import sentencepiece as spm

        self._pairs: dict[str, tuple[Any, Any]] = {}
        for key, directory in (
            ("tr_en", root / "translate-tr_en-1_5"),
            ("en_tr", root / "translate-en_tr-1_5"),
        ):
            tokenizer = spm.SentencePieceProcessor(model_file=str(directory / "sentencepiece.model"))
            translator = ctranslate2.Translator(
                str(directory / "model"), device="cpu", inter_threads=1, intra_threads=2
            )
            self._pairs[key] = (tokenizer, translator)

    def translate(self, text: str, pair: str) -> str:
        tokenizer, translator = self._pairs[pair]
        tokens = tokenizer.encode(text, out_type=str)
        result = translator.translate_batch(
            [tokens], beam_size=4, max_decoding_length=256, repetition_penalty=1.1,
            no_repeat_ngram_size=4,
        )[0]
        return tokenizer.decode(result.hypotheses[0]).strip()


# --------------------------------------------------------------------------
# Text to speech
# --------------------------------------------------------------------------

class PiperBackend:
    """Piper voice kept loaded in-process; falls back to the CLI per sentence."""

    def __init__(self, voice: Path, piper_bin: Path, length_scale: float) -> None:
        self.voice_path = voice
        self.piper_bin = piper_bin
        self.length_scale = length_scale
        self.sample_rate = 22_050
        self.mode = "cli"
        self.load_error: str | None = None
        self._voice: Any = None
        self._lock = threading.Lock()
        try:
            config = json.loads(Path(str(voice) + ".json").read_text(encoding="utf-8"))
            self.sample_rate = int(config["audio"]["sample_rate"])
        except Exception:
            pass
        try:
            from piper import PiperVoice

            self._voice = PiperVoice.load(str(voice))
            self.sample_rate = int(self._voice.config.sample_rate)
            self.mode = "inprocess"
        except Exception as exc:  # piper not importable -> CLI still works
            self.load_error = f"{type(exc).__name__}: {exc}"

    def synthesize(self, text: str) -> np.ndarray:
        """Return int16 mono PCM at ``sample_rate`` (no effects, full scale)."""
        with self._lock:
            if self.mode == "inprocess":
                try:
                    return np.frombuffer(self._synthesize_inprocess(text), dtype="<i2")
                except Exception as exc:
                    self.load_error = f"{type(exc).__name__}: {exc}"
                    self.mode = "cli"
            return self._synthesize_cli(text)

    def _synthesize_inprocess(self, text: str) -> bytes:
        voice = self._voice
        if hasattr(voice, "synthesize_stream_raw"):  # piper-tts 1.2 API
            return b"".join(voice.synthesize_stream_raw(text, length_scale=self.length_scale))
        from piper import SynthesisConfig  # piper1-gpl >= 1.3 API

        config = SynthesisConfig(length_scale=self.length_scale, volume=1.0)
        return b"".join(chunk.audio_int16_bytes for chunk in voice.synthesize(text, syn_config=config))

    def _synthesize_cli(self, text: str) -> np.ndarray:
        with tempfile.TemporaryDirectory(prefix="apex-tts-") as tmp:
            wav = Path(tmp) / "sentence.wav"
            subprocess.run(
                [
                    str(self.piper_bin), "-m", str(self.voice_path), "-f", str(wav),
                    "--length-scale", str(self.length_scale), "--volume", "1.0",
                ],
                input=text.encode("utf-8"),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                check=True,
                timeout=90,
            )
            with wave.open(str(wav), "rb") as source:
                self.sample_rate = source.getframerate()
                return np.frombuffer(source.readframes(source.getnframes()), dtype="<i2")


class SpeechSession:
    """One spoken answer; sentences are queued as soon as they exist."""

    def __init__(self, pipeline: "SpeechPipeline", voice: str = DEFAULT_VOICE) -> None:
        self._pipeline = pipeline
        self._open = True
        self.voice = voice

    def say(self, text: str) -> None:
        clean = speech_text(text)
        if self._open and clean:
            self._pipeline.synth_queue.put((self, clean))

    def finish(self) -> None:
        if self._open:
            self._open = False
            self._pipeline.synth_queue.put((self, None))


class SpeechPipeline:
    """Sentence queue -> render worker -> single raw ``aplay`` stream.

    ``audio_lock`` is taken when the first session starts and released by the
    player thread after the last queued session has finished playing, so an
    alarm and speech can never overlap.
    """

    def __init__(self, engine: "AssistantEngine") -> None:
        self.engine = engine
        self.synth_queue: queue.Queue = queue.Queue()
        self.play_queue: queue.Queue = queue.Queue()
        self._mutex = threading.Lock()
        self._active = 0
        threading.Thread(target=self._synth_loop, name="apex-tts-render", daemon=True).start()
        threading.Thread(target=self._play_loop, name="apex-tts-play", daemon=True).start()

    def begin(self, voice: str = DEFAULT_VOICE) -> SpeechSession | None:
        engine = self.engine
        if not engine.audio_available:
            engine.tts_error = "RuntimeError: USB AUDIO DAC bulunamadı"
            return None
        with self._mutex:
            if self._active == 0 and not engine.audio_lock.acquire(blocking=False):
                return None  # an alarm is playing
            self._active += 1
        return SpeechSession(self, voice)

    def pending(self) -> int:
        return self.synth_queue.qsize() + self.play_queue.qsize()

    def _synth_loop(self) -> None:
        while True:
            session, text = self.synth_queue.get()
            if text is None:
                self.play_queue.put((session, None))
                continue
            try:
                self.play_queue.put((session, self.engine.render_speech(text, session.voice)))
            except Exception as exc:
                self.engine.tts_error = f"{type(exc).__name__}: {exc}"

    def _play_loop(self) -> None:
        player: subprocess.Popen | None = None
        while True:
            _session, pcm = self.play_queue.get()
            if pcm is not None:
                try:
                    if player is None or player.poll() is not None:
                        player = self.engine.open_player()
                    assert player.stdin is not None
                    player.stdin.write(pcm)
                    player.stdin.flush()
                    self.engine.tts_error = None
                except Exception as exc:
                    self.engine.tts_error = f"{type(exc).__name__}: {exc}"
                    player = self._close(player)
                continue
            with self._mutex:
                self._active -= 1
                last = self._active == 0
            if last:
                player = self._close(player)
                self.engine.audio_lock.release()

    @staticmethod
    def _close(player: subprocess.Popen | None) -> None:
        if player is None:
            return None
        try:
            if player.stdin is not None:
                player.stdin.close()
            player.wait(timeout=90)
        except Exception:
            player.kill()
        return None


# --------------------------------------------------------------------------
# Engine
# --------------------------------------------------------------------------

class AssistantEngine:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.vdevice: Any = None
        self.vlm: Any = None
        self.inference_lock = threading.Lock()
        self.history: deque[tuple[str, str]] = deque(maxlen=args.history_messages)
        self.history_lock = threading.Lock()
        self.audio_lock = threading.Lock()
        self.started_at = time.monotonic()
        self.last_inference_s: float | None = None
        self.first_sentence_s: float | None = None
        self.last_camera_used: bool | None = None
        self.last_error: str | None = None
        self.tts_error: str | None = None
        self.translator: OfflineTranslator | None = None
        self.audio_available = False
        self.piper: PiperBackend | None = None
        self.speech: SpeechPipeline | None = None
        self.scene: Any = None
        self.trace: deque[dict[str, Any]] = deque(maxlen=30)
        self.default_voice = pick_voice(getattr(args, "voice_style", DEFAULT_VOICE))
        self.power_lock = threading.Lock()
        self.power_checked_at = 0.0
        self.power_voltage_v: float | None = None
        self.power_flags = 0

    def start(self) -> None:
        from hailo_platform import VDevice
        from hailo_platform.genai import VLM

        params = VDevice.create_params()
        params.group_id = "SHARED"
        self.vdevice = VDevice(params)
        self.vlm = VLM(self.vdevice, str(self.args.hef))
        self.translator = OfflineTranslator(self.args.translation_root)
        self.start_audio()
        self.start_scene()

    def start_scene(self) -> None:
        """Start the always-on person/object detector (optional component)."""
        try:
            from apex_scene import SceneWatcher

            self.scene = SceneWatcher(
                self.args.snapshot_url, self.args.detector,
                interval=self.args.scene_interval, threads=self.args.scene_threads,
                busy=self.inference_lock.locked,
            )
            self.scene.start()
        except Exception as exc:
            self.scene = None
            self.last_error = f"scene: {type(exc).__name__}: {exc}"

    def start_audio(self) -> None:
        try:
            devices = subprocess.run(
                ["aplay", "-l"], capture_output=True, text=True, check=True, timeout=5
            ).stdout
            self.audio_available = "DAC" in devices and "USB Audio" in devices
        except Exception:
            self.audio_available = False
        self.piper = PiperBackend(self.args.voice, self.args.piper, self.args.tts_length_scale)
        self.speech = SpeechPipeline(self)
        # Warm the ONNX session so the first real sentence is not the slow one.
        threading.Thread(target=self._warm_tts, name="apex-tts-warm", daemon=True).start()

    def _warm_tts(self) -> None:
        try:
            self.render_speech("Hazır.")
        except Exception as exc:
            self.tts_error = f"{type(exc).__name__}: {exc}"

    def close(self) -> None:
        if self.vlm is not None:
            try:
                self.vlm.clear_context()
                self.vlm.release()
            except Exception:
                pass
            self.vlm = None
        if self.vdevice is not None:
            try:
                self.vdevice.release()
            except Exception:
                pass
            self.vdevice = None

    def clear_history(self) -> None:
        # Every inference already clears the Hailo context in ``generate``.
        # Resetting the chat must therefore remain a cheap in-memory operation;
        # calling VLM.clear_context() here caused an avoidable PCIe/power spike.
        with self.history_lock:
            self.history.clear()

    def _camera_frame(self) -> tuple[np.ndarray, bool]:
        """Current frame for the VLM: (336x336 RGB array, is_infrared)."""
        from PIL import Image

        from apex_scene import prepare_frame

        request = urllib.request.Request(self.args.snapshot_url, headers={"Cache-Control": "no-cache"})
        try:
            with urllib.request.urlopen(request, timeout=3.0) as response:
                payload = response.read(12 * 1024 * 1024)
        except Exception as exc:
            raise RuntimeError(f"Kamera karesi alınamadı ({exc})") from exc
        with Image.open(io.BytesIO(payload)) as image:
            image, infrared = prepare_frame(image)
            # The model only gets 336x336 pixels.  Squeezing 16:9 into that
            # distorts people and wastes detail, so cut a square around the
            # nearest detected person (or the centre when nobody is found).
            image = image.crop(self._focus_square(*image.size))
            image = image.resize((336, 336), Image.Resampling.BILINEAR)
            # HailoRT writes into its input buffer; Pillow's zero-copy view is
            # read-only, so return an explicit contiguous, writable array.
            return np.array(image, dtype=np.uint8, copy=True, order="C"), infrared

    def _focus_square(self, width: int, height: int) -> tuple[int, int, int, int]:
        people = [d for d in (self.scene.detections() if self.scene is not None else None) or []
                  if d["label"] == "person"]
        side, center_x, center_y = float(height), width / 2.0, height / 2.0
        if people:
            x1, y1, x2, y2 = max(people, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))["box"]
            center_x, center_y = (x1 + x2) / 2 * width, (y1 + y2) / 2 * height
            side = min(float(height), max((x2 - x1) * width, (y2 - y1) * height) * 1.25, float(width))
            side = max(side, height * 0.45)
        left = min(max(center_x - side / 2, 0), width - side)
        top = min(max(center_y - side / 2, 0), height - side)
        return int(left), int(top), int(left + side), int(top + side)

    def scene_line(self) -> str:
        """Detector grounding for the VLM, or "" when unavailable."""
        detections = self.scene.detections() if self.scene is not None else None
        if detections is None:
            return ""
        from apex_scene import summarize

        return f" Your object detector currently reports: {summarize(detections)}."

    def _prompt(
        self, message_en: str, plan: dict[str, Any]
    ) -> tuple[list[dict[str, Any]], list[np.ndarray], str]:
        """Build (prompt, frames, user_text) for one request."""
        intent = plan["intent"]
        content: list[dict[str, str]] = []
        frames: list[np.ndarray] = []
        system = SYSTEM_PROMPTS[intent]
        text = message_en
        if intent == "visual":
            text = plan.get("question_en") or re.sub(
                r"\b(?:on|in|at|from|through)\s+(?:the\s+|your\s+)?camera\b", "in front of you",
                message_en, flags=re.IGNORECASE,
            )
            if plan["use_camera"]:
                frame, infrared = self._camera_frame()
                frames.append(frame)
                content.append({"type": "image"})
                if infrared:
                    system += INFRARED_NOTE
                if "How many people" in text:
                    # Counting people is the detector's job, not the VLM's.
                    people = sum(1 for d in (self.scene.detections() if self.scene else None) or []
                                 if d["label"] == "person")
                    system += f" Your person detector counts {people} person(s) in view; trust this number."
            else:
                system += BLIND_NOTE
        elif intent == "triage":
            topics = medical_topics(plan.get("message_tr", ""))
            if topics:
                system += " The survivor mentions: " + ", ".join(topics) + "."
            if plan["critical"]:
                system += CRITICAL_NOTE
        content.append({"type": "text", "text": text})

        prompt: list[dict[str, Any]] = [{"role": "system", "content": [{"type": "text", "text": system}]}]
        if intent in ("chat", "triage"):
            # Follow-ups need context; a fresh look must not be biased by it.
            with self.history_lock:
                prior = list(self.history)
            for role, past in prior:
                prompt.append({"role": role, "content": [{"type": "text", "text": past[:600]}]})
        prompt.append({"role": "user", "content": content})
        return prompt, frames, f"[SYSTEM] {system}\n[USER] {text}"

    def _translate_sentences(self, sentences: list[str], state: dict[str, Any]) -> Iterator[str]:
        assert self.translator is not None
        for sentence in sentences:
            sentence = clean_en_sentence(sentence)
            key = re.sub(r"\W+", " ", sentence.lower()).strip()
            if len(sentence) < 2 or key in state["seen"]:
                continue  # small VLMs like to repeat themselves
            state["seen"].add(key)
            state["kept"].append(sentence)
            state["count"] += 1
            turkish = polish_tr_sentence(self.translator.translate(sentence, "en_tr"))
            if turkish:
                if state["first_at"] is None:
                    state["first_at"] = time.monotonic()
                yield turkish

    def generate(self, message: str, plan: dict[str, Any] | bool) -> Iterator[str]:
        """Yield the Turkish answer one sentence at a time."""
        if isinstance(plan, bool):  # legacy call: generate(message, use_camera)
            plan = classify(message, "on" if plan else "off")
        if self.vlm is None:
            raise RuntimeError("VLM henüz hazır değil")
        if not self.inference_lock.acquire(blocking=False):
            raise RuntimeError("Yapay zeka başka bir yanıt üretiyor; lütfen bitmesini bekle")

        started = time.monotonic()
        pieces_en: list[str] = []
        answer_tr: list[str] = []
        state: dict[str, Any] = {"seen": set(), "first_at": None, "kept": [], "count": 0}
        buffer = ""
        plan = {**plan, "message_tr": message}
        limit = MAX_SENTENCES.get(plan["intent"], 3)
        stopped = False
        record: dict[str, Any] = {"t": time.strftime("%H:%M:%S"), "question_tr": message, **plan}
        self.trace.append(record)
        try:
            if self.translator is None:
                raise RuntimeError("Çeviri katmanı hazır değil")
            message_en = self.translator.translate(message, "tr_en")
            if plan.get("delay_s"):
                time.sleep(plan["delay_s"])  # "5 sn sonra bak": look after the person is in place
            prompt, frames, user_text = self._prompt(message_en, plan)
            record.update(question_en=message_en, user_text=user_text, frames=len(frames))
            try:
                with self.vlm.generate(
                    prompt=prompt,
                    frames=frames,
                    temperature=0.2 if plan["intent"] == "visual" else 0.35,
                    # A fixed seed made a repeated question return the identical text.
                    seed=int(time.time() * 1000) % 100_000,
                    max_generated_tokens=self.args.max_tokens,
                ) as generation:
                    for raw_chunk in generation:
                        chunk = str(raw_chunk)
                        for token in SPECIAL_TOKENS:
                            chunk = chunk.replace(token, "")
                        if not chunk:
                            continue
                        pieces_en.append(chunk)
                        buffer += chunk
                        sentences, buffer = split_sentences(buffer)
                        for sentence in sentences:
                            # The 2B model loops ("The person is very close to the
                            # person in the centre ..."): stop at the first repeat
                            # or once the answer is long enough.
                            if state["count"] >= limit or looks_repeated(sentence, state["kept"]):
                                stopped = True
                                break
                            for turkish in self._translate_sentences([sentence], state):
                                answer_tr.append(turkish)
                                yield turkish
                        if stopped:
                            buffer = ""
                            break
            except GeneratorExit:
                raise
            except Exception:
                if not stopped:  # aborting a running generation may raise; that is fine
                    raise

            if not stopped:
                sentences, buffer = split_sentences(buffer, final=True)
                for sentence in sentences:
                    if state["count"] >= limit or looks_repeated(sentence, state["kept"]):
                        break
                    for turkish in self._translate_sentences([sentence], state):
                        answer_tr.append(turkish)
                        yield turkish

            answer_en = " ".join(state["kept"]).strip() or "".join(pieces_en).strip()
            record["raw_en"] = "".join(pieces_en).strip()[:1200]
            record["stopped_early"] = stopped
            with self.history_lock:
                self.history.append(("user", (plan.get("question_en") or message_en)[:1000]))
                self.history.append(("assistant", answer_en[:1600]))
            now = time.monotonic()
            self.last_inference_s = now - started
            self.first_sentence_s = (state["first_at"] - started) if state["first_at"] else None
            self.last_camera_used = bool(plan["use_camera"])
            self.last_error = None
            record.update(
                answer_en=answer_en, answer_tr=" ".join(answer_tr),
                seconds=round(self.last_inference_s, 2),
                first_sentence_s=round(self.first_sentence_s, 2) if self.first_sentence_s else None,
            )
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            record["error"] = self.last_error
            raise
        finally:
            try:
                self.vlm.clear_context()
            finally:
                self.inference_lock.release()

    # ---- speech ---------------------------------------------------------

    def render_speech(self, text: str, voice: str | None = None) -> bytes:
        """Text -> robot-filtered, level-normalised s16le mono PCM."""
        if self.piper is None:
            raise RuntimeError("TTS hazır değil")
        raw = self.piper.synthesize(text)
        rate = self.piper.sample_rate
        result = subprocess.run(
            [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-f", "s16le", "-ar", str(rate), "-ac", "1", "-i", "pipe:0",
                "-af", voice_filter(voice or self.default_voice, rate),
                "-f", "s16le", "-ar", str(rate), "-ac", "1", "pipe:1",
            ],
            input=raw.tobytes(),
            capture_output=True,
            check=True,
            timeout=60,
        )
        shaped = np.frombuffer(result.stdout, dtype="<i2")
        levelled = normalize_speech(shaped, rate, self.args.tts_target_dbfs, self.args.tts_peak_dbfs)
        pause = np.zeros(int(rate * 0.09), dtype="<i2")
        return levelled.tobytes() + pause.tobytes()

    def open_player(self) -> subprocess.Popen:
        assert self.piper is not None
        return subprocess.Popen(
            [
                "aplay", "-q", "-D", self.args.audio_device,
                "-t", "raw", "-f", "S16_LE", "-c", "1", "-r", str(self.piper.sample_rate), "-",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def begin_speech(self, voice: str | None = None) -> SpeechSession | None:
        if self.speech is None:
            return None
        return self.speech.begin(pick_voice(voice, self.default_voice))

    def speak_async(self, text: str, voice: str | None = None) -> bool:
        """Queue a whole text for speech, sentence by sentence."""
        session = self.begin_speech(voice)
        if session is None:
            return False
        sentences, _ = split_sentences(text, final=True)
        for sentence in sentences:
            session.say(sentence)
        session.finish()
        return True

    def _speak_locked(self, text: str) -> None:
        """Render and play speech while ``audio_lock`` is already held."""
        if not self.audio_available:
            raise RuntimeError("USB AUDIO DAC bulunamadı")
        pcm = self.render_speech(speech_text(text))
        player = self.open_player()
        player.communicate(input=pcm, timeout=90)
        if player.returncode:
            raise RuntimeError(f"aplay çıkış kodu {player.returncode}")

    def _play_wav(self, path: Path) -> None:
        subprocess.run(
            ["aplay", "-q", "-D", self.args.audio_device, str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            check=True,
            timeout=90,
        )

    def _alert_samples(self, name: str, sample_rate: int = 22_050) -> np.ndarray:
        """Create original, attention-grabbing alerts without sampled media."""
        durations = {"caution": 1.8, "rescue": 2.8, "red_alert": 4.2}
        if name not in durations:
            raise ValueError("Bilinmeyen ses efekti")
        duration = durations[name]
        t = np.arange(int(sample_rate * duration), dtype=np.float64) / sample_rate

        if name == "caution":
            gate = ((t % 0.45) < 0.23).astype(np.float64)
            signal = (np.sin(2 * math.pi * 760 * t) + 0.45 * np.sin(2 * math.pi * 1140 * t)) * gate
        elif name == "rescue":
            signal = np.zeros_like(t)
            for start, freq in ((0.0, 440), (0.42, 660), (0.84, 880), (1.45, 550), (1.87, 825)):
                local = t - start
                mask = (local >= 0) & (local < 0.65)
                envelope = np.exp(-3.4 * np.maximum(local, 0))
                signal += mask * envelope * (
                    np.sin(2 * math.pi * freq * local) + 0.35 * np.sin(2 * math.pi * freq * 2 * local)
                )
        else:
            # Ominous original capital-ship alarm: low accelerating sweep,
            # dissonant harmonic and gated upper siren. No copyrighted sample.
            sweep = 95 + 48 * (0.5 + 0.5 * np.sin(2 * math.pi * 0.23 * t))
            phase = 2 * math.pi * np.cumsum(sweep) / sample_rate
            pulse = 0.62 + 0.38 * np.sign(np.sin(2 * math.pi * 1.15 * t))
            upper = np.sin(2 * math.pi * (510 + 85 * np.sin(2 * math.pi * 0.31 * t)) * t)
            signal = pulse * (np.sin(phase) + 0.48 * np.sin(2.03 * phase) + 0.24 * upper)

        fade = min(int(0.05 * sample_rate), len(signal) // 4)
        if fade:
            ramp = np.linspace(0, 1, fade)
            signal[:fade] *= ramp
            signal[-fade:] *= ramp[::-1]
        peak = float(np.max(np.abs(signal))) or 1.0
        # Alarm level is independent from the speech level (--alert-level).
        return np.asarray(signal / peak * self.args.alert_level * 32767, dtype="<i2")

    def play_alert_async(self, name: str) -> bool:
        if self.audio_lock.locked() or not self.audio_available:
            return False
        threading.Thread(target=self._play_alert, args=(name,), name=f"apex-alert-{name}", daemon=True).start()
        return True

    def _play_alert(self, name: str) -> None:
        if not self.audio_lock.acquire(blocking=False):
            return
        try:
            if name == "red_alert" and self.args.red_alert_file.is_file():
                self._play_wav(self.args.red_alert_file)
                self.tts_error = None
                return
            with tempfile.TemporaryDirectory(prefix="apex-tts-") as tmp:
                wav = Path(tmp) / f"{name}.wav"
                samples = self._alert_samples(name)
                with wave.open(str(wav), "wb") as output:
                    output.setnchannels(1)
                    output.setsampwidth(2)
                    output.setframerate(22_050)
                    output.writeframes(samples.tobytes())
                self._play_wav(wav)
                if name == "rescue":
                    self._speak_locked("Yardım geliyor. Sakin olun.")
            self.tts_error = None
        except Exception as exc:
            self.tts_error = f"{type(exc).__name__}: {exc}"
        finally:
            self.audio_lock.release()

    def health(self) -> dict[str, Any]:
        self._refresh_power_status()
        return {
            "ok": self.vlm is not None,
            "version": VERSION,
            "model": self.args.hef.stem,
            "camera_snapshot": self.args.snapshot_url,
            "camera_default": "auto",
            "live_data_available": False,
            "audio_device": self.args.audio_device,
            "audio_available": self.audio_available,
            "voice_ready": self.args.voice.is_file(),
            "red_alert_ready": self.args.red_alert_file.is_file(),
            "translation_ready": self.translator is not None,
            "voices": {name: preset["label"] for name, preset in VOICES.items()},
            "default_voice": self.default_voice,
            "scene": self.scene.state() if self.scene is not None else {"ok": False, "enabled": False},
            "tts_backend": self.piper.mode if self.piper else None,
            "tts_backend_note": self.piper.load_error if self.piper else None,
            "tts_pending": self.speech.pending() if self.speech else 0,
            "busy": self.inference_lock.locked(),
            "speaking": self.audio_lock.locked(),
            "last_inference_s": self.last_inference_s,
            "first_sentence_s": self.first_sentence_s,
            "last_camera_used": self.last_camera_used,
            "last_error": self.last_error,
            "tts_error": self.tts_error,
            "power_voltage_v": self.power_voltage_v,
            "power_flags": self.power_flags,
            "undervoltage_now": bool(self.power_flags & 0x1),
            "undervoltage_seen": bool(self.power_flags & 0x10000),
            "uptime_s": round(time.monotonic() - self.started_at, 1),
        }

    def _refresh_power_status(self) -> None:
        now = time.monotonic()
        if now - self.power_checked_at < 5 or not self.power_lock.acquire(blocking=False):
            return
        try:
            voltage = subprocess.run(
                ["vcgencmd", "pmic_read_adc", "EXT5V_V"],
                capture_output=True, text=True, timeout=3, check=False,
            ).stdout
            match = re.search(r"=([0-9.]+)V", voltage)
            if match:
                self.power_voltage_v = float(match.group(1))
            throttled = subprocess.run(
                ["vcgencmd", "get_throttled"],
                capture_output=True, text=True, timeout=3, check=False,
            ).stdout
            match = re.search(r"0x([0-9a-fA-F]+)", throttled)
            if match:
                self.power_flags = int(match.group(1), 16)
            self.power_checked_at = now
        except Exception:
            self.power_checked_at = now  # vcgencmd missing (not a Pi)
        finally:
            self.power_lock.release()


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

class AssistantHandler(BaseHTTPRequestHandler):
    server_version = f"APEX-Assistant/{VERSION}"

    @property
    def engine(self) -> AssistantEngine:
        return self.server.engine  # type: ignore[attr-defined]

    @property
    def index_html(self) -> bytes:
        return self.server.index_html  # type: ignore[attr-defined]

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(HTTPStatus.OK, "text/html; charset=utf-8", self.index_html)
        elif path == "/api/health":
            self._json(HTTPStatus.OK, self.engine.health())
        elif path == "/api/scene":
            scene = self.engine.scene
            self._json(HTTPStatus.OK, scene.state() if scene is not None else {"ok": False, "enabled": False})
        elif path == "/api/trace":
            self._json(HTTPStatus.OK, {"trace": list(self.engine.trace)})
        else:
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/api/reset":
            self.engine.clear_history()
            self._json(HTTPStatus.OK, {"ok": True})
            return
        if path == "/api/sound":
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                data = json.loads(self.rfile.read(length))
                name = str(data.get("name", ""))
                if name not in {"caution", "rescue", "red_alert"}:
                    raise ValueError("Bilinmeyen ses efekti")
                accepted = self.engine.play_alert_async(name)
                if not accepted:
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "Ses aygıtı meşgul veya bağlı değil"})
                else:
                    self._json(HTTPStatus.ACCEPTED, {"ok": True, "name": name})
            except (ValueError, TypeError, AttributeError, json.JSONDecodeError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        if path == "/api/say":
            # Speak a text directly (voice try-out and operator announcements).
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 8192)
                data = json.loads(self.rfile.read(length))
                text = str(data.get("text", "")).strip()[:600]
                if not text:
                    raise ValueError("Boş metin")
                if self.engine.speak_async(text, data.get("voice")):
                    self._json(HTTPStatus.ACCEPTED, {"ok": True})
                else:
                    self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "Ses aygıtı meşgul veya bağlı değil"})
            except (ValueError, TypeError, AttributeError, json.JSONDecodeError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        if path != "/api/chat":
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return

        try:
            length = min(int(self.headers.get("Content-Length", "0")), 32_768)
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("Geçersiz istek")
            message = str(data.get("message", "")).strip()
            if not message:
                raise ValueError("Boş mesaj gönderilemez")
            if len(message) > 2000:
                raise ValueError("Mesaj 2000 karakterden uzun olamaz")
            camera_mode = parse_camera_mode(data)
            speak = bool(data.get("speak", False))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        plan = classify(message, camera_mode)
        use_camera = plan["use_camera"]

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("Connection", "close")
        self.end_headers()

        answer: list[str] = []
        session = self.engine.begin_speech(data.get("voice")) if speak else None
        try:
            phase = {
                "triage": "DURUM DEĞERLENDİRME" + (" · KRİTİK" if plan["critical"] else ""),
                "visual": "SAHNE ANALİZİ",
                "live": "CANLI VERİ YOK",
                "chat": "YANIT ÜRETİLİYOR",
            }[plan["intent"]] + (" · KAMERA" if use_camera else " · KAMERASIZ") + (
                f" · {int(plan['delay_s'])} SN SONRA BAKILACAK" if plan.get("delay_s") else "")
            self._stream_json({
                "type": "phase", "text": phase, "camera_used": use_camera,
                "intent": plan["intent"], "critical": plan["critical"],
            })
            for sentence in self.engine.generate(message, plan):
                self._stream_json({"type": "delta", "text": (" " if answer else "") + sentence})
                answer.append(sentence)
                if session is not None:
                    session.say(sentence)
            self._stream_json({
                "type": "done",
                "text": " ".join(answer).strip(),
                "seconds": self.engine.last_inference_s,
                "first_sentence_seconds": self.engine.first_sentence_s,
                "camera_used": use_camera,
                "camera_mode": camera_mode,
                "intent": plan["intent"],
                "critical": plan["critical"],
                "source": "vlm",
            })
        except Exception as exc:
            self._stream_json({"type": "error", "error": str(exc)})
        finally:
            if session is not None:
                session.finish()

    def _stream_json(self, payload: dict[str, Any]) -> None:
        try:
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        self._send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False).encode())

    def _send(self, status: HTTPStatus, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args: object) -> None:
        if self.path.split("?", 1)[0] != "/api/health":
            super().log_message(fmt, *args)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=os.environ.get("APEX_AI_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("APEX_AI_PORT", "8090")))
    parser.add_argument("--hef", type=Path, default=Path(os.environ.get(
        "APEX_AI_HEF", "/usr/local/hailo/resources/models/hailo10h/Qwen2-VL-2B-Instruct.hef"
    )))
    parser.add_argument("--snapshot-url", default=os.environ.get(
        "APEX_AI_SNAPSHOT_URL", "http://127.0.0.1:8888/snapshot"
    ))
    parser.add_argument("--voice", type=Path, default=Path(os.environ.get(
        "APEX_AI_VOICE", "/opt/apex/models/piper/tr_TR-dfki-medium.onnx"
    )))
    parser.add_argument("--piper", type=Path, default=Path(os.environ.get(
        "APEX_AI_PIPER", "/opt/apex/venvs/assistant/bin/piper"
    )))
    parser.add_argument("--audio-device", default=os.environ.get(
        "APEX_AI_AUDIO_DEVICE", "plughw:CARD=DAC,DEV=0"
    ))
    parser.add_argument("--red-alert-file", type=Path, default=Path(os.environ.get(
        "APEX_AI_RED_ALERT_FILE", "/opt/apex/assistant/audio/red_alert.wav"
    )))
    parser.add_argument("--index", type=Path, default=Path(os.environ.get(
        "APEX_AI_INDEX", "/opt/apex/assistant/index.html"
    )))
    parser.add_argument("--max-tokens", type=int, default=int(os.environ.get("APEX_AI_MAX_TOKENS", "140")))
    parser.add_argument("--detector", type=Path, default=Path(os.environ.get(
        "APEX_AI_DETECTOR", str(Path(__file__).resolve().parent / "models" / "yolox_tiny.onnx")
    )), help="YOLOX ONNX model for the always-on person/object detector")
    parser.add_argument("--scene-interval", type=float, default=float(os.environ.get(
        "APEX_AI_SCENE_INTERVAL", "1.0"
    )), help="seconds between detector passes")
    parser.add_argument("--scene-threads", type=int, default=int(os.environ.get("APEX_AI_SCENE_THREADS", "2")))
    parser.add_argument("--voice-style", choices=sorted(VOICES), default=os.environ.get(
        "APEX_AI_VOICE_STYLE", DEFAULT_VOICE
    ) if os.environ.get("APEX_AI_VOICE_STYLE", DEFAULT_VOICE) in VOICES else DEFAULT_VOICE)
    parser.add_argument("--history-messages", type=int, default=6)
    parser.add_argument("--translation-root", type=Path, default=Path(os.environ.get(
        "APEX_AI_TRANSLATION_ROOT", "/opt/apex/models/translate"
    )))
    parser.add_argument("--tts-target-dbfs", type=float, default=float(os.environ.get(
        "APEX_AI_TTS_TARGET_DBFS", "-17"
    )), help="speech level as gated RMS in dBFS (about -16/-15 LUFS)")
    parser.add_argument("--tts-peak-dbfs", type=float, default=float(os.environ.get(
        "APEX_AI_TTS_PEAK_DBFS", "-1"
    )), help="speech peak ceiling in dBFS")
    parser.add_argument("--tts-length-scale", type=float, default=float(os.environ.get(
        "APEX_AI_TTS_LENGTH_SCALE", "1.0"
    )))
    parser.add_argument("--alert-level", type=float, default=float(os.environ.get(
        "APEX_AI_ALERT_LEVEL", "0.38"
    )), help="peak amplitude (0..1) of the generated caution/rescue alerts")
    parser.add_argument("--render-test", type=Path, metavar="WAV",
                        help="render --render-text to WAV, print its level and exit (no Hailo needed)")
    parser.add_argument("--render-text", default="Merhaba, ben APEX. Ses seviyesi ve anlaşılırlık testi yapıyorum.")
    return parser.parse_args(argv)


def render_test(args: argparse.Namespace) -> int:
    engine = AssistantEngine(args)
    engine.piper = PiperBackend(args.voice, args.piper, args.tts_length_scale)
    started = time.monotonic()
    pcm = np.frombuffer(engine.render_speech(args.render_text, args.voice_style), dtype="<i2")
    elapsed = time.monotonic() - started
    rate = engine.piper.sample_rate
    with wave.open(str(args.render_test), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(rate)
        output.writeframes(pcm.tobytes())
    rms, peak = pcm_stats(pcm, rate)
    print(json.dumps({
        "wav": str(args.render_test), "backend": engine.piper.mode, "sample_rate": rate,
        "audio_s": round(len(pcm) / rate, 2), "render_s": round(elapsed, 2),
        "rms_dbfs": round(rms, 1), "peak_dbfs": round(peak, 1),
    }, ensure_ascii=False))
    return 0


def main() -> int:
    args = parse_args()
    if args.render_test:
        return render_test(args)
    for required in (args.hef, args.index):
        if not required.is_file():
            raise FileNotFoundError(required)
    engine = AssistantEngine(args)
    engine.start()
    server = ThreadingHTTPServer((args.host, args.port), AssistantHandler)
    server.daemon_threads = True
    server.engine = engine  # type: ignore[attr-defined]
    server.index_html = args.index.read_bytes()  # type: ignore[attr-defined]
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        server.server_close()
        engine.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
