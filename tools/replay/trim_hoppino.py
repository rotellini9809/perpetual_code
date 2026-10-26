"""Rebuild the replay excerpt from the untouched original flight log."""
import argparse
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
ORIGINAL=ROOT/'datasets/hoppino_VOLO_UFFICIALE_original.csv'
OUTPUT=ROOT/'datasets/hoppino_VOLO_UFFICIALE.csv'
START_MS,END_MS=2000,35000


def generate():
    lines=ORIGINAL.read_text(encoding='utf-8-sig').splitlines()
    rows=[line for line in lines[1:] if START_MS<=int(line.split(',',1)[0])<END_MS]
    origin=int(rows[0].split(',',1)[0])
    # Preserve every sensor/estimate field verbatim; only rebase timestamps.
    return '\n'.join([lines[0]]+[str(int(line.split(',',1)[0])-origin)+','+line.split(',',1)[1] for line in rows])+'\n'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--check',action='store_true')
    args=parser.parse_args(); content=generate()
    if args.check:
        if OUTPUT.read_text(encoding='utf-8-sig')!=content: parser.exit(1,'Hoppino excerpt is stale; run tools/replay/trim_hoppino.py\n')
        print('Hoppino excerpt matches the original log (2..35 s).')
    else: OUTPUT.write_text(content,encoding='utf-8',newline='\n')
