% COMPILEKORTEXMEX - Generates MEX file to interface MATLAB with the C++ Kortex
%   Specifies all the options as the CMakeLists.txt file
%   Copyright 2019 Kinova Inc.

%% global definition
[str,maxsize,endian] = computer;

cppCompiler = mex.getCompilerConfigurations('c++','selected');
cppCompilerName = cppCompiler.ShortName();

if ismac()
    disp('mac os not supported yet');
end
    
if ispc()
    if (cppCompilerName == 'MSVCPP150')
        CPP_SUB_FOLDER = 'win_msvc_x86-64';
    else
        disp(['unknown c++ compiler name: ' cppCompilerName]);
    end
elseif isunix()
    CPP_SUB_FOLDER = 'linux_gcc_x86-64';
end


BUILD_MODE = 'Release';
%BUILD_MODE = 'Debug';

%% mex-wrapper directory will be output directory so the MEX file will be generated there.
mainDir = fullfile(pwd,'..');

%% Define source files
% MEX entry file
mexInterfaceFile = fullfile(mainDir,'mex-wrapper', 'src', 'kortexApiMexInterface.cpp');

% API wrapper file
wrapperFile = fullfile(mainDir,'mex-wrapper', 'src', 'kortexApiWrapper.cpp');
converterFile = fullfile(mainDir,'mex-wrapper', 'src', 'proto_converter.cpp');
mexConverterFile = fullfile(mainDir, 'mex-wrapper', 'src', 'mexConverter.cpp');


%% Define compiler directives 
if ispc()
    cdirective1 = '-D_OS_WINDOWS';
    useconsole = '-D_START_CONSOLE_';
elseif isunix()
    cdirective1 = '-D_OS_UNIX';
    useconsole = '';
end

%% Define include directories
% Kortex API:
kortexApiIncludeDir = fullfile(mainDir, 'simplified_api', 'install', 'package', 'include');
protobufIncludeDir = fullfile(mainDir,'simplified_api', 'kortex_api', 'cpp', CPP_SUB_FOLDER, 'include');

protobuf_client_includes = ['-I' fullfile(protobufIncludeDir,'client')];
protobuf_client_stubs = ['-I' fullfile(protobufIncludeDir,'client_stubs')];
protobuf_common = ['-I' fullfile(protobufIncludeDir,'common')];
protobuf_messages = ['-I' fullfile(protobufIncludeDir,'messages')];
ipath_api = ['-I' kortexApiIncludeDir];
protobuf_path = ['-I' protobufIncludeDir];

% MATLAB Mex wrapper
ipath_mex = ['-I' fullfile(mainDir, 'mex-wrapper', 'include')];


%% Define link directories
% Kortex library  
lpath_api = ['-L' fullfile(mainDir, 'simplified_api', 'install', 'package', 'lib', BUILD_MODE)];


%% Define libraries
lib_api = '-lSimplifiedApi';

if ispc()
    lib2 = '-lwinMM';
    lib3 = '-lws2_32';
elseif isunix()
    lib2 = '';
    lib3 = '';
end


%% Target compile options (target_compile_options(${PROJECT_NAME} PRIVATE /MT))
if ispc()
    compilerOptions = '/MT';
elseif isunix()
    compilerOptions = '';
end


%% Compile and generate mex file 
mex('-v',...
    '-g',...
    '-R2018a',...
    cdirective1, useconsole, ...
    ipath_api, protobuf_path, protobuf_client_includes, protobuf_client_stubs, ...
    protobuf_common, protobuf_messages, ipath_mex,...
    lpath_api, ...
    lib_api, lib2, lib3, ...
    mexInterfaceFile, wrapperFile, converterFile, mexConverterFile, ...
    ['COMPFLAGS="$COMPFLAGS ' compilerOptions '"']); 
