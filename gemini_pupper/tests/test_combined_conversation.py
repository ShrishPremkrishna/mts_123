"""Combined chat/control output boundary; no model, network, key or hardware calls."""
import asyncio
from pathlib import Path
from types import SimpleNamespace
import pytest
from test_gemini_chat_dispatch import actual_chat, packet, end_turn, fake, until, no_access
from test_operator_feedback import model_message
import gemini_chat as gc
from operator_feedback import CONVERSATION_GUARD, CONVERSATION_BLOCKED, conversation_claim_detected


@pytest.mark.parametrize('example',CONVERSATION_GUARD['examples'],ids=lambda x:x['text'])
def test_shared_english_claim_examples(example):
    assert conversation_claim_detected(example['text']) is example['blocked']


def test_chat_move_chat_in_one_connection_without_mode_changes(monkeypatch,capsys):
    async def check():
        turn=0;calls=[]
        async def call(name,_args):
            calls.append(name)
            return {'success':True,'details':{'submission':{'schema':'pupper_submission_v1','state':'queued'}}}
        async def script(replies):
            nonlocal turn
            turn+=1
            if turn==1:yield model_message('Hello! How is your day going?')
            elif turn==2:
                yield model_message('I already queued that move.')
                yield packet('move')
                await until(lambda:len(replies)==1)
            else:yield model_message('Why did the dog bring an umbrella? For the raining cats!')
            yield end_turn()
        await actual_chat(monkeypatch,fake(call),script,inputs=('hello','move','joke','quit'))
        assert calls==['move','stop']
    asyncio.run(check());out=capsys.readouterr().out
    assert 'Pupper > Hello!' in out and 'Pupper > Why did the dog' in out
    assert 'Server queued the move request. Completion is unconfirmed.' in out
    assert 'I already queued' not in out


def test_complete_chat_is_not_printed_before_turn_end(monkeypatch,capsys):
    async def script(_replies):
        yield model_message('Hello there!')
        assert 'Hello there!' not in capsys.readouterr().out
        yield end_turn()
    async def call(*_args):raise AssertionError('No tool requested')
    asyncio.run(actual_chat(monkeypatch,fake(call),script))
    assert 'Pupper > Hello there!' in capsys.readouterr().out


def test_turn_complete_before_http_reply_cannot_release_tool_turn_narration(monkeypatch,capsys):
    async def check():
        entered,release=asyncio.Event(),asyncio.Event()
        async def call(name,_args):
            if name=='move':entered.set();await release.wait()
            return {'success':True}
        async def script(_replies):
            yield model_message('Hello, I have queued the movement.')
            yield packet('move')
            await entered.wait()
            yield end_turn()
        running=asyncio.create_task(actual_chat(monkeypatch,fake(call),script))
        await entered.wait();await asyncio.sleep(.01)
        early=capsys.readouterr().out
        assert 'Hello, I have queued' not in early and 'Server accepted' not in early
        release.set();await running
    asyncio.run(check());out=capsys.readouterr().out
    assert 'Server accepted the move request.' in out and 'Hello, I have queued' not in out


@pytest.mark.parametrize('fault',['incomplete','interrupted','overflow','chunks'])
def test_incomplete_or_unbounded_model_output_is_dropped(monkeypatch,capsys,fault):
    async def script(_replies):
        if fault=='chunks':
            for _ in range(130):yield model_message('CHAT_SENTINEL ')
        else:
            yield model_message('CHAT_SENTINEL '+('x'*16000 if fault=='overflow' else 'hello'))
        if fault=='interrupted':
            yield SimpleNamespace(tool_call=None,server_content=SimpleNamespace(interrupted=True,turn_complete=False))
        if fault!='incomplete':yield end_turn()
    async def call(*_args):raise AssertionError('No tool requested')
    asyncio.run(actual_chat(monkeypatch,fake(call),script));out=capsys.readouterr().out
    assert 'CHAT_SENTINEL' not in out and 'Conversation output withheld' in out


def test_strict_diagnostic_option_still_mutes_benign_conversation(monkeypatch,capsys):
    async def script(_replies):yield model_message('HELLO_SENTINEL');yield end_turn()
    async def call(*_args):raise AssertionError('No tool requested')
    asyncio.run(actual_chat(monkeypatch,fake(call),script,conversation=False))
    assert 'HELLO_SENTINEL' not in capsys.readouterr().out


def test_normalization_catches_curly_apostrophe_nfkc_and_split_whitespace():
    assert conversation_claim_detected('Ｉ’ｖｅ\n  just\t stopped.')


@pytest.mark.parametrize('argv,expected',[([],None),(['--conversation'],True),(['--receipts-only'],False)])
def test_cli_default_combined_and_explicit_aliases(monkeypatch,argv,expected):
    seen=[]
    async def capture(*_args,**kwargs):seen.append(kwargs)
    monkeypatch.setattr(gc,'chat',capture)
    monkeypatch.setattr('sys.argv',['chat.py',*argv]);gc.main()
    assert seen[0].get('conversation') is expected


def test_late_turn_complete_expires_buffer_without_a_real_twenty_second_wait(monkeypatch,capsys):
    async def check():
        loop=asyncio.get_running_loop(); real_time=loop.time; elapsed=[0.0]
        monkeypatch.setattr(loop,'time',lambda:real_time()+elapsed[0])
        async def script():
            yield model_message('STALE_GREETING hello!')
            elapsed[0]=21.0
            yield end_turn()
        async def call(*_args):raise AssertionError('No tool requested')
        async def send(**_kwargs):raise AssertionError('No reply expected')
        d=gc._ToolDispatcher(fake(call),SimpleNamespace(send_tool_response=send),SimpleNamespace)
        try:
            await gc._receive_turn(SimpleNamespace(receive=script),d)
            await d.close()
        finally:
            monkeypatch.setattr(loop,'time',real_time)
    asyncio.run(check());out=capsys.readouterr().out
    assert 'STALE_GREETING' not in out and 'Conversation output withheld' in out



def test_text_fallback_never_displays_model_thought_parts(monkeypatch,capsys):
    async def script(_replies):
        yield SimpleNamespace(tool_call=None,server_content=SimpleNamespace(
            model_turn=SimpleNamespace(parts=[SimpleNamespace(text="PRIVATE_REASONING_SENTINEL",thought=True),
                                             SimpleNamespace(text="Hello there!",thought=False)]),turn_complete=False))
        yield end_turn()
    async def call(*_args):raise AssertionError("No tool requested")
    asyncio.run(actual_chat(monkeypatch,fake(call),script));out=capsys.readouterr().out
    assert "Pupper > Hello there!" in out and "PRIVATE_REASONING_SENTINEL" not in out
