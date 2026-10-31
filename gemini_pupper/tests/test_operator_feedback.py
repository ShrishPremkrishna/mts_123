"""Operator statements come from matched receipts, never arbitrary model prose."""
import asyncio
from types import SimpleNamespace
import pytest
from test_gemini_chat_dispatch import actual_chat, packet, end_turn, fake, until, no_access
import gemini_chat as gc


def model_message(text):
    return SimpleNamespace(tool_call=None, server_content=SimpleNamespace(
        output_transcription=SimpleNamespace(text=text), turn_complete=False,
        model_turn=SimpleNamespace(parts=[SimpleNamespace(text=text, inline_data=SimpleNamespace(data='synthetic-audio'))])))


def test_default_suppresses_split_no_tool_acceptance_claims(monkeypatch, capsys):
    async def script(_replies):
        yield model_message('I have ')
        yield model_message('queued the move, and I am standing still.')
        yield end_turn()
    async def call(*_args):
        raise AssertionError('No tool was requested')
    asyncio.run(actual_chat(monkeypatch, fake(call), script))
    out=capsys.readouterr().out
    assert 'queued the move' not in out and 'standing still' not in out
    assert 'This voice turn submitted no robot commands.' in out


def test_claim_before_rejection_never_becomes_operator_acceptance(monkeypatch, capsys):
    async def check():
        pending=asyncio.Event(); calls=[]
        async def call(name,_args):
            calls.append(name)
            if name=='move':
                await pending.wait()
                return {'success':False,'message':'RAW_FALSE_BODY_CLAIM'}
            return {'success':True}
        async def script(replies):
            yield model_message('I queued your move already.')
            yield packet('move')
            await until(lambda: calls==['move'])
            early = capsys.readouterr().out
            assert 'Server accepted' not in early and 'I queued your move' not in early
            yield model_message('My body moved ten metres.')
            pending.set()
            await until(lambda: len(replies)==1)
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script)
    asyncio.run(check())
    out=capsys.readouterr().out
    assert 'Server reported the move request failed.' in out
    assert 'RAW_FALSE_BODY_CLAIM' not in out and 'ten metres' not in out
    assert 'Server accepted the move' not in out


@pytest.mark.parametrize('result,expected,forbidden',[
 ({'success':True},'Server accepted the move request. Completion is unconfirmed.','queued'),
 ({'success':True,'details':{'submission':{'schema':'pupper_submission_v1','state':'queued'}}},'Server queued the move request. Completion is unconfirmed.','completed'),
 ({'success':True,'details':{'submission':{'schema':'pupper_submission_v1','state':'already_active'}}},'Server accepted the move request. Completion is unconfirmed.','queued'),
 ({'success':True,'details':{'submission':{'schema':'other','state':'queued'}}},'Server accepted the move request. Completion is unconfirmed.','queued'),
 ({'success':False},'Server reported the move request failed. Earlier effects are not ruled out.','nothing happened'),
 ({'success':'true'},'The move response is unknown. Check server status and the robot.','accepted'),
 ({'success':True,'outcome_unknown':True},'The move response is unknown. Check server status and the robot.','accepted'),
 ({'success':True,'superseded':True},'This request was superseded. Earlier execution is possible.','accepted'),
])
def test_closed_receipt_templates(result,expected,forbidden):
    from operator_feedback import receipt_feedback
    text=receipt_feedback('move',result)
    assert text==expected and forbidden not in text


def test_raw_names_messages_and_historical_outcomes_do_not_authorize_claims():
    from operator_feedback import receipt_feedback
    result={'success':True,'message':'BODY_STANDING','details':{'last_outcome':{'name':'move','success':True,'message':'completed'}}}
    assert receipt_feedback('INJECTED_NAME',result)=='Server accepted the robot request. Completion is unconfirmed.'
    assert receipt_feedback('move',result,current=False)=='This request was superseded. Earlier execution is possible.'
    assert receipt_feedback('move',result,dispatched=False)=='The move request was not dispatched.'
    marker={'success':True,'details':{'submission':{'schema':'pupper_submission_v1','state':'queued'}}}
    assert 'queued' not in receipt_feedback('robot_status',marker)
    assert receipt_feedback('stop',marker)=='Server accepted the Stop request. Completion is unconfirmed. Physical stopping is unverified.'


@pytest.mark.parametrize("old_name", ["move", "robot_status"])
def test_stop_invalidates_old_feedback_even_when_old_backend_returns_success(capsys, old_name):
    async def check():
        entered,release=asyncio.Event(),asyncio.Event()
        async def call(name,_args):
            if name==old_name:entered.set();await release.wait()
            return {'success':True,'message':'RAW_OLD_ACCEPTANCE'}
        async def send(**_kwargs):pass
        d=gc._ToolDispatcher(fake(call),SimpleNamespace(send_tool_response=send),SimpleNamespace)
        d.submit(packet(old_name).tool_call.function_calls);await entered.wait()
        d.submit(packet('stop').tool_call.function_calls)
        await until(lambda:d.pending_stops==0)
        release.set();await d.drain();await d.close()
    asyncio.run(check());out=capsys.readouterr().out
    assert 'superseded' in out and f'Server accepted the {"move" if old_name == "move" else "status"} request' not in out
    assert 'RAW_OLD_ACCEPTANCE' not in out


def test_conversation_locally_blocks_unadvertised_motion_and_unlock(capsys):
    async def check():
        calls=[]
        async def call(name,_args):calls.append(name);return {'success':True}
        async def send(**_kwargs):pass
        d=gc._ToolDispatcher(fake(call),SimpleNamespace(send_tool_response=send),SimpleNamespace,conversation=True)
        d.submit(packet('move','unlock','robot_status').tool_call.function_calls)
        await d.drain();await d.close()
        assert calls==['robot_status']
    asyncio.run(check());out=capsys.readouterr().out
    assert 'The move request was not dispatched.' in out and 'The unlock request was not dispatched.' in out


def test_non_dispatch_marker_overrides_queued_even_with_success():
    from operator_feedback import receipt_feedback
    r = {"success": True, "not_dispatched": True, "details": {"submission": {
        "schema": "pupper_submission_v1", "state": "queued"}}}
    assert receipt_feedback("move", r) == "The move request was not dispatched."
    r["outcome_unknown"] = True
    assert receipt_feedback("move", r) == "The move response is unknown. Check server status and the robot."


def test_actual_conversation_config_filters_tools_and_labels_prose(monkeypatch, capsys):
    from google import genai
    from contextlib import asynccontextmanager
    seen, called, replies = [], [], []
    async def call(name, _args):
        called.append(name)
        return {"success": True}
    async def receive():
        yield model_message("I already queued a move.\nMy body is standing.")
        yield packet("move", "unlock", "robot_status")
        await until(lambda: len(replies) == 3)
        yield end_turn()
    async def send_tool_response(**kwargs):replies.extend(kwargs["function_responses"])
    async def send_client_content(**_kwargs):pass
    @asynccontextmanager
    async def connect(**setup):
        seen.append(setup)
        yield SimpleNamespace(receive=receive, send_client_content=send_client_content,
                              send_tool_response=send_tool_response)
    monkeypatch.setattr(genai, "Client", lambda **_kwargs: SimpleNamespace(aio=SimpleNamespace(live=SimpleNamespace(connect=connect))))
    inputs=iter(("talk", "quit"));monkeypatch.setattr("builtins.input",lambda _p:next(inputs))
    b=fake(call);b.setup=lambda:("Unchanged prompt",[{"name":n,"description":"test"} for n in ["move","unlock","robot_status","stop"]])
    asyncio.run(gc.chat(b,gc.MODEL,False,conversation=True))
    declared=seen[0]["config"].tools[0].function_declarations
    assert [t.name for t in declared]==["robot_status","stop"]
    assert called==["robot_status"]
    output=capsys.readouterr().out
    assert "UNVERIFIED CONVERSATION" in output
    assert "Unverified conversation > I already queued" in output
    assert "\nUnverified conversation > My body" in output
    assert "The move request was not dispatched." in output


def test_show_is_explicit_diagnostic_not_an_operator_mode(capsys):
    gc.show("RAW_PROMPT",[{"name":"RAW_TOOL"}],True)
    out=capsys.readouterr().out
    assert "DIAGNOSTIC prompt (not operator evidence)" in out
    assert "END DIAGNOSTIC CONFIGURATION" in out


@pytest.mark.parametrize("reason", ["pending", "unknown", "invalidated", "missing", "health_error"])
def test_local_robot_backend_refusal_is_not_reported_as_a_server_reply(monkeypatch, reason):
    from operator_feedback import receipt_feedback
    b=gc.RobotBackend("http://offline.invalid")
    b._command_epoch="fake:0"
    if reason=="pending": b._stops_pending=1
    if reason=="unknown": b._stop_unknown=True
    if reason=="invalidated": b._epoch_invalidated=True
    def forbidden(*_args):raise AssertionError("Mutation POST must not occur")
    monkeypatch.setattr(b,"_post",forbidden)
    if reason in ("missing","health_error"):
        b._command_epoch=None
        def health(_path):
            if reason=="health_error":raise TimeoutError("synthetic metadata failure")
            return {}
        monkeypatch.setattr(b,"_get",health)
    result=asyncio.run(b.call("move",{}))
    assert result["not_dispatched"] is True
    assert receipt_feedback("move",result)=="The move request was not dispatched."


@pytest.mark.parametrize("flag", ["superseded", "outcome_unknown", "not_dispatched"])
@pytest.mark.parametrize("bad_value", [{}, [], None, 0, "false"])
def test_malformed_receipt_flags_never_authorize_acceptance(flag, bad_value):
    from operator_feedback import receipt_feedback
    assert receipt_feedback("move", {"success": True, flag: bad_value}) == "The move response is unknown. Check server status and the robot."



def test_thrown_status_transport_error_is_unknown_not_a_server_failure(monkeypatch, capsys):
    async def call(name, _args):
        assert name == "robot_status"
        raise ConnectionError("RAW_TRANSPORT_DETAIL")
    async def script(replies):
        yield packet("robot_status")
        await until(lambda: len(replies) == 1)
        yield end_turn()
    asyncio.run(actual_chat(monkeypatch, fake(call), script))
    out = capsys.readouterr().out
    assert "The status response is unknown. Check server status and the robot." in out
    assert "Server reported" not in out and "RAW_TRANSPORT_DETAIL" not in out
