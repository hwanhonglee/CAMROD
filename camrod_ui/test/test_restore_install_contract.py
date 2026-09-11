"""HH_260911 - Keep localized responses and checkout-specific install paths testable."""
from pathlib import Path
import os, json, subprocess
ROOT=Path(__file__).resolve().parents[2]
def test_frontend_sync_resolves_actual_checkout_without_creating_output(tmp_path):
    script=ROOT/'camrod_ui/scripts/sync_frontend_build.sh'
    target=tmp_path/'outputs'
    result=subprocess.run([str(script),'--print-paths'],cwd='/tmp',env={**os.environ,'CAMROD_BUILD_ROOT':str(target)},text=True,capture_output=True,check=True)
    paths=dict(line.split('=',1) for line in result.stdout.splitlines())
    assert paths['SOURCE_ROOT']==str(ROOT)
    assert paths['WS_ROOT']==str(target)
    assert not target.exists()

def test_installed_voice_component_exports_library_path():
    cmake=(ROOT/'camrod_voice/CMakeLists.txt').read_text()
    assert 'ament_export_libraries(${PROJECT_NAME}_component)' in cmake
    assert 'install(DIRECTORY resource/' in cmake

def test_dock_copy_does_not_alter_acknowledgement_or_error_codes():
    source=(ROOT/'camrod_ui/camrod_ui_robot/assets/frontend/src/TelemetryWorkspace.js').read_text()
    helper=source[source.index('export async function postDockingRequest('):source.index('export function parkingPolicyMessage(')].replace('export ','')
    script=helper+'''\n(async()=>{const calls=[];const results=[];
      for(const code of ['docking_requested','already_charging','parking_in_progress']){
        const success=code!=='parking_in_progress';const body={success,action:success?code:undefined,error:success?undefined:code,message:'English backend message'};
        try{results.push(await postDockingRequest(async(url,options)=>{calls.push({url,options});return {ok:success,json:async()=>body};}));}
        catch(error){results.push({message:error.message,code:body.error});}
      } console.log(JSON.stringify({calls,results}));})();'''
    result=subprocess.run(['node'],input=script,text=True,capture_output=True,check=True)
    output=json.loads(result.stdout)
    assert output['calls']==[{'url':'/ui/dock','options':{'method':'POST'}}]*3
    assert output['results'][0]['action']=='docking_requested' and output['results'][0]['success'] is True
    assert output['results'][1]['message']=='이미 충전 중입니다.'
    assert output['results'][2]['code']=='parking_in_progress'
    assert output['results'][2]['message'].startswith('주차 또는 충전 동작이 진행 중입니다.')
