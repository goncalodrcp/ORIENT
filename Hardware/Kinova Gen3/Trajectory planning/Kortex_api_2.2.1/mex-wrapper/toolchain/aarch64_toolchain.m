function [tc, results] = aarch64_toolchain()
toolchain.Platforms  = {computer('arch')};
toolchain.Versions   = {'1.0'};
toolchain.Artifacts  = {'gmake'};
toolchain.FuncHandle = str2func('getToolchainInfoFor');
toolchain.ExtraFuncArgs = {};

[tc, results] = coder.make.internal.generateToolchainInfoObjects(mfilename, toolchain);
end

function tc = getToolchainInfoFor(platform, version, artifact, varargin)
% Toolchain Information

tc = coder.make.ToolchainInfo('BuildArtifact', 'gmake makefile', 'SupportedLanguages', {'Asm/C/C++'});
tc.Name = 'Aarch64 - ARM 64 | gmake makefile';
tc.Platform = platform;

% MATLAB setup
%tc.MATLABSetup = 'robotic.internal.addCompilerPath();';

% Toolchain's attribute
tc.addAttribute('TransformPathsWithSpaces');
tc.addAttribute('SupportsUNCPaths',     false);
tc.addAttribute('SupportsDoubleQuotes', false);

tc.addIntrinsicMacros({'ANSI_OPTS', 'CPP_ANSI_OPTS'});

if ispc
    tc.addMacro('SHELL', '%SystemRoot%/system32/cmd.exe');
end
tc.addMacro('CCOUTPUTFLAG', '--output_file=');
tc.addMacro('LDOUTPUTFLAG', '--output_file=');
% tc.addMacro('CPFLAGS', '-O binary');

% Assembler
assembler = tc.getBuildTool('Assembler');
assembler.setName(['Aarch64 ', version, ' Assembler']);
if ispc
   assembler.setCommand('aarch64-linux-gnu-as.exe');
else
   assembler.setCommand('aarch64-linux-gnu-as'); 
end
assembler.setDirective('IncludeSearchPath', '-I');
assembler.setDirective('PreprocessorDefine', '-D');
assembler.setDirective('OutputFlag', '-o');
assembler.setDirective('Debug', '-g');
assembler.DerivedFileExtensions = {'Object'};
assembler.setFileExtension('Source','.s');
assembler.setFileExtension('Object', '.s.o');

% Compiler
compiler = tc.getBuildTool('C Compiler');
compiler.setName(['Aarch64 ', version, ' C Compiler']);
if ispc
   %compiler.setPath('%AARCH64_PATH%');
   compiler.setCommand('aarch64-linux-gnu-gcc.exe');
else
   compiler.setCommand('aarch64-linux-gnu-gcc-6'); 
end
compiler.setDirective('IncludeSearchPath', '-I');
compiler.setDirective('PreprocessorDefine', '-D');
compiler.setDirective('OutputFlag', '-o');
compiler.setDirective('Debug', '-g');
compiler.setFileExtension('Source', '.c');
compiler.setFileExtension('Header', '.h');
compiler.setFileExtension('Object', '.c.o');

% C++ compiler
cppcompiler = tc.getBuildTool('C++ Compiler');
cppcompiler.setName(['Aarch64 ', version, ' C++ Compiler']);
if ispc
   % cppcompiler.setPath('%AARCH64_PATH%');
   cppcompiler.setCommand('aarch64-linux-gnu-g++.exe');
else
   cppcompiler.setCommand('aarch64-linux-gnu-g++-6'); 
end
cppcompiler.setDirective('IncludeSearchPath', '-I');
cppcompiler.setDirective('PreprocessorDefine', '-D');
cppcompiler.setDirective('OutputFlag', '-o');
cppcompiler.setDirective('Debug', '-g');
cppcompiler.setFileExtension('Source', '.cpp');
cppcompiler.setFileExtension('Header', '.hpp');
cppcompiler.setFileExtension('Object', '.cpp.o');

% Linker
linker = tc.getBuildTool('Linker');
linker.setName(['Aarch64 ', version, ' Linker']);
if ispc
   % linker.setPath('%AARCH64_PATH%');
   linker.setCommand('aarch64-linux-gnu-gcc.exe');
else
   linker.setCommand('aarch64-linux-gnu-gcc-6'); 
end
linker.setDirective('Library', '-l');
linker.setDirective('LibrarySearchPath', '-L');
linker.setDirective('OutputFlag', '-o');
linker.setDirective('Debug', '-g');
linker.setFileExtension('Executable', '.elf');
linker.setFileExtension('Shared Library', '.so');
linker.Libraries = {'-lm'};

% C++ Linker
cpplinker = tc.getBuildTool('C++ Linker');
cpplinker.setName(['Aarch64 ', version, ' C++ Linker']);
if ispc
   %%cpplinker.setPath('%AARCH64_PATH%');
   cpplinker.setCommand('aarch64-linux-gnu-g++.exe');
else
   cpplinker.setCommand('aarch64-linux-gnu-g++-6'); 
end
cpplinker.setDirective('Library', '-l');
cpplinker.setDirective('LibrarySearchPath', '-L');
cpplinker.setDirective('OutputFlag', '-o');
cpplinker.setDirective('Debug', '-g');
cpplinker.setFileExtension('Executable', '');
cpplinker.setFileExtension('Shared Library', '.so');
cpplinker.Libraries = {'-lm'};

% Archiver
archiver = tc.getBuildTool('Archiver');
archiver.setName(['Aarch64 ', version, ' Archiver']);
if ispc
   %archiver.setPath('%AARCH64_PATH%');
   archiver.setCommand('aarch64-linux-gnu-ar.exe');
else
   archiver.setCommand('aarch64-linux-gnu-ar'); 
end
archiver.setDirective('OutputFlag', '');
archiver.setFileExtension('Static Library', '.a');

tc.setBuilderApplication(platform);

% --------------------------------------------
% BUILD CONFIGURATIONS
% --------------------------------------------
optimsOffOpts = {'-O0'};
optimsOnOpts = {'-O2'};
cCompilerOpts = {''};
archiverOpts = {'-r'};

compilerOpts = {...
    '-c',...
    };

cppCompilerOpts  = {'-c $(CPP_ANSI_OPTS)'};

cLinkerOpts = {'-Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" $(LDDEBUG)'};
cppLinkerOpts = {'-Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" $(CPPLDDEBUG)'};

assemblerOpts = compilerOpts;

% Get the debug flag per build tool
debugFlag.CCompiler   = '-g -D"_DEBUG"';
debugFlag.CppCompiler   = '-g -D"_DEBUG"';
debugFlag.Linker      = '-g';
debugFlag.Archiver    = '-g';

cfg = tc.getBuildConfiguration('Faster Builds');
cfg.setOption('Assembler',  horzcat(cCompilerOpts, assemblerOpts, '$(ASFLAGS_ADDITIONAL)', '$(INCLUDES)'));
cfg.setOption('C Compiler', horzcat(cCompilerOpts, compilerOpts, optimsOffOpts));
cfg.setOption('C++ Compiler', horzcat(cppCompilerOpts,  optimsOffOpts));
cfg.setOption('Linker',     cLinkerOpts);
cfg.setOption('C++ Linker', cppLinkerOpts);
cfg.setOption('Archiver',   archiverOpts);

cfg = tc.getBuildConfiguration('Faster Runs');
cfg.setOption('Assembler',  horzcat(cCompilerOpts, assemblerOpts, '$(ASFLAGS_ADDITIONAL)', '$(INCLUDES)'));
cfg.setOption('C Compiler', horzcat(cCompilerOpts, compilerOpts, optimsOnOpts));
cfg.setOption('C++ Compiler', horzcat(cppCompilerOpts, compilerOpts, optimsOnOpts));
cfg.setOption('Linker',     cLinkerOpts);
cfg.setOption('C++ Linker', cppLinkerOpts);
cfg.setOption('Archiver',   archiverOpts);

cfg = tc.getBuildConfiguration('Debug');
cfg.setOption('Assembler',  horzcat(cCompilerOpts, assemblerOpts, '$(ASFLAGS_ADDITIONAL)', '$(INCLUDES)', debugFlag.CCompiler));
cfg.setOption('C Compiler', horzcat(cCompilerOpts, compilerOpts, optimsOffOpts, debugFlag.CCompiler));
cfg.setOption('C++ Compiler',horzcat(cppCompilerOpts, optimsOffOpts, debugFlag.CppCompiler));
cfg.setOption('Linker',     horzcat(cLinkerOpts, debugFlag.Linker));
cfg.setOption('C++ Linker', cppLinkerOpts);
cfg.setOption('Archiver',   horzcat(archiverOpts, debugFlag.Archiver));

tc.setBuildConfigurationOption('all', 'Download',  '');
tc.setBuildConfigurationOption('all', 'Execute',   '');
tc.setBuildConfigurationOption('all', 'Make Tool', '-f $(MAKEFILE)');

end