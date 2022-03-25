%% Y4ArenaSetup version 22.3.17 by Rishika Mohanta
%% Note: Try to do this before the flies are put in the arena

%% Conventions:
%% =============
%% Color    MFCId       ArenaIdx    Quadrant
%% Green    0           1           0001
%% Red      1           2           0010
%% Blue     2           3           1000
%% Grey     3           4           0100
%%
%% Follow the following order for the masks:
%% Arena 0: Arm 0, Arm 1, Arm 2; Arena 1: Arm 0, Arm 1, Arm 2; Arena 2: Arm 0, Arm 1, Arm 2; Arena 3: Arm 0, Arm 1, Arm 2
%% The position of each arm is defined here: 
%% https://janelia-experimental-technology.github.io/y-arena/sa/module/
%% For consistency with MATLAB, arenas and arms are referred using indices 1, 2 and 3 except when sending commands to the MFCs.  
%%
%% The odor map convention is as follows:
%% 0: Air (No odor solution in vial 0)
%% 1: Odor 1 (Odor 1 solution in vial 1) (Typically OCT)
%% 2: Odor 2 (Odor 2 solution in vial 2) (Typically MCH)

% Initialize the Code Environment for the Arena
clc; imaqreset; clear;

% Y4ArenaDefaultSettings; % Use only if you want to use the default settings

% Load the settings from the file
load('Z:\Rishika\4Y-Maze\Y4ArenaSettings.mat');

try
    newojs = instrfind; fclose(instrfind); % Use only if last session didn't close properly
catch
end
%% Initialize the Arena LED Controllers

narenas = 16;
narms = 3*narenas;

% Setting up port for LED control
serial_port_for_LED_Controller = 'COM3'; % COM1 ON 4Y COM3 ON 1Y
hLEDController = serial(serial_port_for_LED_Controller, 'BaudRate', 115200, 'Terminator', 'CR');
fopen(hLEDController);
hComm.hLEDController = hLEDController;

% Initialize the Arena IR Backlight
olfactoryArena_LED_control(hLEDController, 'IR', IRIntensity);
newojs = instrfind;

%% Capture the Background Image

imaqreset;

% Background Parameters
BkgdMovieLength = 30; % seconds

% Initialize the Arena Camera
vidobj = videoinput('mwspinnakerimaq', 1, 'Mono8'); % Initialize the camera
triggerconfig(vidobj, 'manual'); % Set the trigger mode to manual
vidobj.FramesPerTrigger = 66 * BkgdMovieLength;

% Start Capture
tic;
start(vidobj); % Start the camera
trigger(vidobj); % Trigger the camera
wait(vidobj, 1.5 * BkgdMovieLength); % Wait for the camera to finish capturing
disp(['Background video captured for ' num2str(toc) ' seconds'])
% Stop Capture

BkgdMovie = getdata(vidobj, 66 * BkgdMovieLength); % Get the captured video
BkgdMovie = squeeze(BkgdMovie); % Remove the singleton dimension
BackgroundImage = max(BkgdMovie, [], 3); % Do a Maximum Intensity Projection of the movie
BackgroundImage = imcomplement(BackgroundImage); % Invert the image

% Close the camera
flushdata(vidobj)
delete(vidobj)
clear vidobj BkgdMovie

% Display the Background Image
imshow(BackgroundImage)

%% Load arm and reward masks
reward_distance = 0.8;

if redrawmasks
    drawMask;
    if savemasks
        save(allMaskDirectory,'AllMasks','MaskStack','RewardMaskStack')
    end
else
    load(allMaskDirectory);
end
% Clean any overlaps in the masks
cleanMask;

%% Setup Odor Controller

% Connect to valve board
yArenaNode = ros2node('y_arena_matlab_node');
arenaOdorsPub = ros2publisher(yArenaNode, '/arena_odors', 'y_arena_interfaces/ArenaOdors');

% Make sure all arenas are receiving only air
arenaOdorsMsg = ros2message('y_arena_interfaces/ArenaOdors');

% Arena 1
arenaOdorsMsg.arena = uint8(3);
arenaOdorsMsg.odors = uint8([0 0 0]);
send(arenaOdorsPub, arenaOdorsMsg);

% Arena 2
arenaOdorsMsg.arena = uint8(1);
arenaOdorsMsg.odors = uint8([0 0 0]);
send(arenaOdorsPub, arenaOdorsMsg);

% Arena 3
arenaOdorsMsg.arena = uint8(2);
arenaOdorsMsg.odors = uint8([0 0 0]);
send(arenaOdorsPub, arenaOdorsMsg);

% Arena 4
arenaOdorsMsg.arena = uint8(3);
arenaOdorsMsg.odors = uint8([0 0 0]);
send(arenaOdorsPub, arenaOdorsMsg);

%% SETUP COMPLETE


% pause(10)
% possibilities = perms([0 1 2]);%combvec(0:2,0:2,0:2)';
% for i=1:length(possibilities)
%    arenaOdorsMsg.arena = uint8(3);
%    arenaOdorsMsg.odors = uint8([possibilities(i,1) possibilities(i,2) possibilities(i,3)]); 
%    send(arenaOdorsPub, arenaOdorsMsg);
%    pause(1)
% end
