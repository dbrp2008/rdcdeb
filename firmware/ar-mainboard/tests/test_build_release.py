#!/usr/bin/env python3
"""Exercise the actual hidden VS Code builds with CMake/Ninja; no board or programmer."""
import json
import os
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory

root = Path(__file__).resolve().parents[1]
tasks = json.loads((root / '.vscode/tasks.json').read_text())['tasks']
build = next(t for t in tasks if t['label'] == 'Build AR MainBoard (Release)')
bench = next(t for t in tasks if t['label'] == 'Build AR Encoder Bench')
cmake = shutil.which(build['command'])
ninja = shutil.which('ninja')
assert cmake and ninja, 'This separate task test requires CMake 3.22+ and Ninja'
exe = '.exe' if os.name == 'nt' else ''

with TemporaryDirectory(prefix='AR task check ') as tmp:
    tmp = Path(tmp)
    board = tmp / 'board with spaces'
    (board / 'tools').mkdir(parents=True)
    shutil.copy2(root / build['args'][1], board / build['args'][1])
    shutil.copy2(root / 'CMakePresets.json', board)
    (board / 'cmake').mkdir()
    (board / 'cmake/gcc-arm-none-eabi.cmake').write_text('# Preset path; probe needs no cross compiler.\n')
    # Small native stand-in: GCC answers the target query; objcopy records the
    # PATH seen by a child of Ninja. Real Arm linking is tested separately.
    probe = tmp / ('probe' + exe)
    source = tmp / 'probe.c'
    source.write_text('''#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int n, char **v) {
    if (n != 2) return 1;
    if (!strcmp(v[1], "-dumpmachine")) { puts("arm-none-eabi"); return 0; }
    FILE *f = fopen(v[1], "w"); if (!f) return 2;
    fputs(getenv("PATH"), f); fclose(f);
    return getenv("FAIL_BUILD") != NULL;
}''')
    subprocess.run(['gcc', str(source), '-o', str(probe)], check=True)
    (board / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.22)
project(Probe LANGUAGES NONE)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_GENERATOR STREQUAL "Ninja")
    message(FATAL_ERROR "wrong configuration")
endif()
file(WRITE "${CMAKE_BINARY_DIR}/configured" "$ENV{PATH}")
if(DEFINED ENV{FAIL_CONFIGURE})
    message(FATAL_ERROR "intentional configure failure")
endif()
if(AR_IMAGE STREQUAL "Drive")
    add_custom_target(probe ALL COMMAND arm-none-eabi-objcopy "${CMAKE_BINARY_DIR}/compiled" VERBATIM)
elseif(AR_IMAGE STREQUAL "EncoderBench")
    # Not an ALL target: only building the named bench target writes the marker.
    add_custom_target(EncoderBench COMMAND arm-none-eabi-objcopy "${CMAKE_BINARY_DIR}/compiled" VERBATIM)
else()
    message(FATAL_ERROR "wrong image")
endif()
''')
    host = tmp / 'host'; host.mkdir()
    shutil.copy2(ninja, host / ('ninja' + exe))
    # Do not accidentally discover a real host cross compiler or ST bundles.
    env = dict(os.environ, PATH=str(host), AR_GCC_BIN='', CUBE_BUNDLE_PATH='',
               LOCALAPPDATA=str(tmp / 'Local AppData'), SystemDrive=str(tmp / 'drive'))
    if os.name == 'nt':
        env['PATH'] += os.pathsep + str(Path(env['SystemRoot']) / 'System32')
    for key in ('CMAKE_PROGRAM_PATH', 'FAIL_BUILD', 'FAIL_CONFIGURE'):
        env.pop(key, None)
    output = board / 'build/Release'; output.mkdir(parents=True)
    stale = output / 'MainBoard.elf'; stale.write_text('old image')
    def task(*args, which=build, ok=True, **override):
        r = subprocess.run([cmake, *args, *which['args']], cwd=board, env={**env, **override},
                           text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        assert (r.returncode == 0) == ok, r.stdout
        assert stale.read_text() == 'old image'  # the helper never deletes or flashes it
        return r.stdout

    def install(directory, complete=True):
        directory.mkdir(parents=True)
        shutil.copy2(probe, directory / ('arm-none-eabi-gcc' + exe))
        if complete:
            shutil.copy2(probe, directory / ('arm-none-eabi-objcopy' + exe))
        return directory

    def used(directory, tree=output):
        for phase in ('configured', 'compiled'):
            first = (tree / phase).read_text().split(os.pathsep)[0]
            assert Path(first).samefile(directory), (phase, first, directory)

    assert 'AR_GCC_BIN' in task(ok=False) and not (output / 'configured').exists()
    assert '-DAR_BUILD_PRESET=EncoderBench' in task(which=bench, ok=False)
    assert not (board / 'build/EncoderBench').exists()
    bundles = Path(env['LOCALAPPDATA']) / 'stm32cube/bundles/gnu-tools-for-stm32'
    install(bundles / '9.1/bin'); newer = install(bundles / '15.2/bin')
    partial = install(bundles / '14.3.1+st.2/bin', complete=False)
    assert 'arm-none-eabi-objcopy' in task(f'-DAR_GCC_BIN={partial}', ok=False)
    task(); used(newer)  # default per-user store, unpinned version, spaces and no compiler on PATH
    task(LOCALAPPDATA=''); used(newer)  # successful hint survives in the cache
    custom = install(tmp / 'custom compiler/bin')
    task(f'-DAR_GCC_BIN={custom}'); used(custom)
    (output / 'compiled').unlink()
    assert 'intentional configure failure' in task(ok=False, FAIL_CONFIGURE='1')
    assert not (output / 'compiled').exists()  # configure failure never invokes Ninja
    task(ok=False, FAIL_BUILD='1'); used(custom)  # Ninja failure reaches VS Code as nonzero
    # Both hidden steps must fail closed even with a stale ELF. A new bench
    # tree can reuse Release's custom compiler hint but cannot build into it.
    bench_out = board / 'build/EncoderBench'; bench_out.mkdir()
    bench_stale = bench_out / 'EncoderBench.elf'; bench_stale.write_text('old bench')
    release_state = {p: p.stat().st_mtime_ns for p in (output / 'compiled', output / 'configured', stale)}
    task(which=bench, LOCALAPPDATA=''); used(custom, bench_out)
    (bench_out / 'compiled').unlink()
    assert 'intentional configure failure' in task(which=bench, ok=False, FAIL_CONFIGURE='1')
    assert not (bench_out / 'compiled').exists()
    task(which=bench, ok=False, FAIL_BUILD='1'); used(custom, bench_out)
    assert bench_stale.read_text() == 'old bench'
    assert all(p.stat().st_mtime_ns == ts for p, ts in release_state.items())
    assert 'Unknown AR_BUILD_PRESET' in task('-DAR_BUILD_PRESET=encoder-typo', ok=False)
    print('EncoderBench task: actual presets, separate tree, cached off-PATH compiler, failed configure/build reject flash dependency PASS')
    # Each remaining case starts with no previous configure hint. Test ST's
    # bounded install layouts, not a recursive search of the developer's PC.
    st = Path(env['SystemDrive']) / 'ST'
    for directory, override in [
        (tmp / 'bundles/gnu-tools-for-stm32/14.3.1+st.2/bin', {'CUBE_BUNDLE_PATH': str(tmp / 'bundles')}),
        (st / 'STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.win32/tools/bin', {}),
        (st / 'STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin', {}),
        (tmp / 'on PATH', {'PATH': str(tmp / 'on PATH') + os.pathsep + env['PATH']}),
    ]:
        shutil.rmtree(output); output.mkdir(); stale.write_text('old image')
        install(directory); task(LOCALAPPDATA='', **override); used(directory)
        shutil.rmtree(directory)
    print('AR Release task: bundle/CLT/IDE/PATH and explicit override, spaced paths, cache retry, fail-closed configure/build PASS')

# Configure the board's actual CMakeLists and presets. Only CubeMX's generated
# subdirectory/toolchain are stand-ins: this host check inspects source selection,
# not MCU compilation. A bench build must not even compile the motor driver.
with TemporaryDirectory(prefix='AR image graph ') as tmp:
    board = Path(tmp)
    for name in ('CMakeLists.txt', 'CMakePresets.json'):
        shutil.copy2(root / name, board)
    shutil.copytree(root / 'Core/Src', board / 'Core/Src')
    mx = board / 'cmake/stm32cubemx'; mx.mkdir(parents=True)
    (mx / 'CMakeLists.txt').write_text('add_library(stm32cubemx INTERFACE)\n'
        'target_sources(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/Core/Src/main.c")\n')
    gcc = Path(shutil.which('gcc')).as_posix()
    (mx.parent / 'gcc-arm-none-eabi.cmake').write_text(f'set(CMAKE_C_COMPILER "{gcc}")\n'
        f'set(CMAKE_ASM_COMPILER "{gcc}")\n'
        'set(CMAKE_OBJCOPY "${CMAKE_COMMAND}")\n')
    def configure(*args, error=None):
        r = subprocess.run([cmake, *args], cwd=board, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        assert (not r.returncode if error is None else r.returncode and error in r.stdout), r.stdout

    bench_sources = {'encoder_bench.c', 'encoder_monitor.c'}  # + enc_capture.c, shared by both images
    drive_sources = {'calibration_app.c', 'ir_setup.c', 'side_ir.c', 'elcp_motor.c', 'elcp_zero_pwm.c',
                     'function1.c', 'function2.c', 'function3.c', 'drive_plan.c', 'enc_watch.c', 'enc_capture.c', 'wheel_spin.c'}
    for preset in ('Debug', 'Release', 'EncoderBench'):
        configure('--preset', preset)
        tree = board / 'build' / preset
        files = {Path(c['file']).name for c in json.loads((tree / 'compile_commands.json').read_text())}
        if preset == 'EncoderBench':
            assert files == bench_sources | {'lcd.c', 'main.c', 'enc_capture.c'}, files  # no adc.c: the line IRs are digital
        else:
            assert drive_sources <= files and not files & bench_sources, files
    configure('--preset', 'Release', '-B', 'build/EncoderBench', error='This build directory is for EncoderBench')
    configure('--preset', 'EncoderBench', '-B', 'build/Release', error='This build directory is for Drive')
    configure('--preset', 'EncoderBench', '-DAR_ENCODER_BENCH=ON', error='AR_ENCODER_BENCH is retired')
    print('Board CMake: disjoint drive/bench sources and cache guards, historic powered bench rejected PASS')
