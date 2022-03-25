% Initialize the User Settings for the Arena

rigName = 'Y16-Arena1';
activeArenas = [true true true true true true true true true true true true true true true true];

% LED Parameters
IRIntensity = 25; % in percent (for IR Backlight)

RedIntensity1 = 30;                              % Intensity of red LED (in percent)
GreenIntensity1 = 0;                             % Intensity of green LED (in percent)
BlueIntensity1 = 0;                              % Intensity of blue LED (in percent)
PulseWidth1 = 500;                               % Width of LED pulse (in ms)
PulsePeriod1 = 1000;                             % Period of LED pulse (in ms)
PulseNumber1 = 1;                                % Number of pulses
Wait1 = 0;                                       % Delay before pulse (in ms)
Off1 = 0;                                        % Dead time after pulse (in ms)
Iterations1 = 1;                                 % Number of times the pulse train is repeated
OptoColor1 = 'RED';                              % RED for CSChrimson, GREEN for Chronos

RedIntensity2 = 30;                              % Intensity of red LED (in percent)
GreenIntensity2 = 0;                             % Intensity of green LED (in percent)
BlueIntensity2 = 0;                              % Intensity of blue LED (in percent)
PulseWidth2 = 500;                               % Width of LED pulse (in ms)
PulsePeriod2 = 1000;                             % Period of LED pulse (in ms)
PulseNumber2 = 1;                                % Number of pulses
Wait2 = 0;                                       % Delay before pulse (in ms)
Off2 = 0;                                        % Dead time after pulse (in ms)
Iterations2 = 1;                                 % Number of times the pulse train is repeated
OptoColor2 = 'RED';                              % RED for CSChrimson, GREEN for Chronos

% Mask Parameters
redrawmasks = false;
savemasks = false;
allMaskDirectory = 'Z:\Rishika\4Y-Maze\AllMasks16.mat';

% Trialwise Odor Delivery Parameters
Odor1ID = 'OCT';
Odor2ID = 'MCH';
NumTrialsPerBlock = [1 2];                      % List of number of trials per block
Odor1_RewardProbability = [0 0.5];                  % Reward probability for Odor1 in each Block
Odor2_RewardProbability = [0 0.5];                  % Reward probability for Odor2 in each Block
baited = true;                                    % Set to true if you want to bait trials

% Runtime Parameters
verbose = 1;                                      % Set to true to show debug information
showflies = true;                                 % Set to true to show fly positions in every frame
savedir = 'Z:\Rishika\4Y-Maze\RunData\';                  % Directory to save data to
addlcomments = '';                                % Additional Comments
expname = 'Y16-Arena1-Test';
fly1genotype = 'WT';
fly2genotype = 'WT';
fly3genotype = 'WT';
fly4genotype = 'WT';
fly5genotype = 'WT';
fly6genotype = 'WT';
fly7genotype = 'WT';
fly8genotype = 'WT';
fly9genotype = 'WT';
fly10genotype = 'WT';
fly11genotype = 'WT';
fly12genotype = 'WT';
fly13genotype = 'WT';
fly14genotype = 'WT';
fly15genotype = 'WT';
fly16genotype = 'WT';
exptime = datestr(now,'mm_dd_yyyy-HH_MM');
earlystop = false;

save("Z:\Rishika\16Y-Maze\Y16ArenaState.mat","earlystop")
save("Z:\Rishika\16Y-Maze\Y16ArenaSettings.mat", "rigName", "activeArenas", "IRIntensity", "RedIntensity1", "GreenIntensity1", "BlueIntensity1", "PulseWidth1", "PulsePeriod1", "PulseNumber1", "Wait1", "Off1", "Iterations1", "OptoColor1", "RedIntensity2", "GreenIntensity2", "BlueIntensity2", "PulseWidth2", "PulsePeriod2", "PulseNumber2", "Wait2", "Off2", "Iterations2", "OptoColor2", "redrawmasks", "savemasks", "allMaskDirectory", "Odor1ID", "Odor2ID", "NumTrialsPerBlock", "Odor1_RewardProbability", "Odor2_RewardProbability", "baited", "verbose", "showflies", "savedir", "addlcomments", "expname", "fly1genotype", "fly2genotype", "fly3genotype", "fly4genotype", "fly5genotype", "fly6genotype", "fly7genotype", "fly8genotype", "fly9genotype", "fly10genotype", "fly11genotype", "fly12genotype", "fly13genotype", "fly14genotype", "fly15genotype", "fly16genotype", "exptime")