import json
import os
from pathlib import Path
import time

def mark(run,step,title,cn,en,**extra):
    run=Path(run)
    value={'step':step,'title':title,'cn':cn,'en':en,'wall':time.time(),**extra}
    temporary=run/'story_state.tmp'
    temporary.write_text(json.dumps(value,ensure_ascii=False),encoding='utf-8')
    os.replace(temporary,run/'story_state.json')
    with (run/'story_events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(value,ensure_ascii=False)+'\n')
