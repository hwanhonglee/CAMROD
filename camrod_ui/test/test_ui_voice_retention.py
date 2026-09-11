"""HH_260911 - Prevent lost Korean UI copy and missing/replaced audio assets."""
from pathlib import Path
import hashlib
import json
import re
import wave
import pytest
ROOT = Path(__file__).resolve().parents[2]
FRONT = ROOT / 'camrod_ui/camrod_ui_robot/assets/frontend/src'
AUDIO = ROOT / 'camrod_voice/resource/audio'
PRESERVED = json.loads((Path(__file__).parent/'data/preserved_audio_sha256.json').read_text())
COPY = json.loads((Path(__file__).parent/'data/korean_copy_contract.json').read_text())
@pytest.mark.parametrize('file,old,new',[(f,o,n) for f,m in COPY.items() for o,n in m.items()])
def test_restored_presentation_is_present(file, old, new):
    source=(FRONT/file).read_text()
    assert old not in source
    # A CARLA camera label may be passed through a variable instead of inline JSX.
    required = new[len('label="'):-1] if new.startswith('label="') and new.endswith('"') else new
    assert required in source
@pytest.mark.parametrize('relative,sha256',list(PRESERVED.items()))
def test_existing_recording_is_never_regenerated(relative, sha256):
    assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==sha256
@pytest.mark.parametrize('number',range(1,14))
def test_site_request_resolves_to_pcm_audio(number):
    manifest=json.loads((AUDIO/'site_announcements.json').read_text())
    entry=next(c for c in manifest['clips'] if c['key']==f'navigation.site_B{number}')
    path=AUDIO/entry['file']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256']
    with wave.open(str(path),'rb') as wav:
        assert (wav.getnchannels(),wav.getframerate(),wav.getsampwidth())==(1,24000,2)
        assert 0.2<wav.getnframes()/wav.getframerate()<5
def test_statically_requested_voice_keys_have_assets():
    keys=set()
    for relative in ['camrod_voice/src/voice_event_policy.py','camrod_voice/src/voice_announcer_node.cpp']:
        keys.update(re.findall(r'\b(?:navigation|safety|battery|system|docking)\.[A-Za-z_0-9]+',(ROOT/relative).read_text()))
    missing=[k for k in sorted(keys) if not any((AUDIO/'ko-KR'/Path(k.replace('.','/')+e)).is_file() for e in ['.wav','.mp3'])]
    assert not missing, missing

def test_original_charge_and_arrival_messages_are_retained():
    source=(FRONT/'App.js').read_text()
    for text in ['도착 완료 · 복귀','충전 완료','충전 연결 대기 중','배달 서비스 및 호출 서비스 이용이 가능합니다.']:
        assert text in source
    guest=(ROOT/'camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html').read_text()
    assert '도착 완료' in guest

def test_site_plus_departure_cue_fits_original_voice_timeout():
    def duration(path):
        with wave.open(str(path),'rb') as wav:
            return wav.getnframes()/wav.getframerate()
    generic=duration(AUDIO/'ko-KR/navigation/to_campsite.wav')
    for i in range(1,14):
        assert duration(AUDIO/f'ko-KR/navigation/site_B{i}.wav')+generic<12.0
