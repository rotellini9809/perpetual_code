"""Small incremental g++ builder shared by host tests and navigation replay."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# Same language level as the firmware (Arduino-Pico: -std=gnu++17, GCC 14); strict ISO here.
FLAGS = ('-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic', '-Iinclude')


class HostBuild:
    def __init__(self, compiler='g++', jobs=4, rebuild=False, root=ROOT):
        self.root = Path(root).resolve()
        self.directory = self.root / '.pio/tests'
        self.compiler = shutil.which(compiler)
        if not self.compiler:
            raise RuntimeError(f'C++ compiler not found: {compiler}')
        banner = subprocess.check_output([self.compiler, '--version'], text=True).splitlines()[0]
        self.version = [banner, Path(self.compiler).stat().st_mtime_ns]
        self.jobs, self.rebuild = jobs, rebuild
        self.compiled = self.linked = 0

    def _relative(self, path):
        return Path(path).relative_to(self.root).as_posix()

    def _fresh(self, output, command, inputs):
        metadata = output.with_suffix(output.suffix + '.json')
        try:
            stamp = output.stat().st_mtime_ns
            return (not self.rebuild and json.loads(metadata.read_text()) == [self.version, command]
                    and all(path.stat().st_mtime_ns <= stamp for path in inputs))
        except (OSError, ValueError):
            return False

    def _run(self, output, command):
        output.parent.mkdir(parents=True, exist_ok=True)
        try:
            result = subprocess.run(command, cwd=self.root, text=True, capture_output=True, check=True)
        except subprocess.CalledProcessError as error:
            # Failed compiler/linker output must never be reused as a cache hit.
            output.unlink(missing_ok=True)
            raise RuntimeError(f'{output.name}:\n{error.stdout}{error.stderr}') from error
        if result.stderr:
            print(result.stderr, end='')
        output.with_suffix(output.suffix + '.json').write_text(json.dumps([self.version, command]))

    def compile(self, source):
        source = Path(source)
        if not source.is_absolute(): source = self.root / source
        relative = self._relative(source)
        output = self.directory / 'obj' / (relative + '.o')
        dependency = output.with_suffix('.d')
        command = [self.compiler, *FLAGS, '-MMD', '-MF', self._relative(dependency),
                   '-c', relative, '-o', self._relative(output)]
        try:
            # g++ make dependencies use relative '/' paths on Windows too.
            rule = dependency.read_text().replace('\\\n', '').splitlines()[0]
            inputs = [self.root / name for name in shlex.split(rule.split(': ', 1)[1])]
        except (OSError, ValueError, IndexError):
            inputs = []
        if not inputs or not self._fresh(output, command, inputs):
            self._run(output, command)
            return output, True
        return output, False

    def build_many(self, targets):
        """Compile each shared source once, then link requested executables."""
        sources = list(dict.fromkeys(source for values in targets.values() for source in values))
        with ThreadPoolExecutor(max_workers=self.jobs) as pool:
            objects = dict(zip(sources, pool.map(self.compile, sources)))
        self.compiled += sum(changed for _, changed in objects.values())
        results = {}
        for name, sources in targets.items():
            inputs = [objects[source][0] for source in sources]
            output = self.directory / 'bin' / (name + '.exe')
            command = [self.compiler, *(self._relative(path) for path in inputs), '-o', self._relative(output)]
            if not self._fresh(output, command, inputs):
                self._run(output, command)
                self.linked += 1
            results[name] = output
        return results

    def build(self, name, sources):
        return self.build_many({name: sources})[name]
