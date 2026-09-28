"""Create a portable project bundle, excluding scratch runs and caches."""
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]


def main():
    output=ROOT/'output/amr_rescue_project.zip'
    output.parent.mkdir(parents=True,exist_ok=True)
    directories=('config','controllers','docs','protos','sar','scripts','tests','tools','worlds','output/validation')
    files=[ROOT/name for name in ('README.md','requirements.txt','requirements-dev.txt','.gitignore')]
    for directory in directories:
        files.extend(p for p in (ROOT/directory).rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.suffix!='.pyc' and not p.name.startswith('.'))
    with ZipFile(output,'w',ZIP_DEFLATED) as archive:
        for path in sorted(set(files)):
            archive.write(path,Path('amr_rescue')/path.relative_to(ROOT))
    print(output)


if __name__=='__main__': main()
