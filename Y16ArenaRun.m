%% Y4Arena Probabilistic Experiment Run Code version 22.3.17 by Rishika Mohanta

%%% Terminology:
%% Panel: Each panel has 4 y-mazes
%% Arena: Each single y-maze is a single arena.
%% Arm: Each Arena has 3 arms.
%% Home Arm: The Arm of each Arena that is recieving air.

imaqreset;

%% Calculated Variables for trial structure
MaxFrames = 10 * 60 * 60 * 3;                     % maximum size of session (10 frames/sec x 60 secs/min x 60 min/hr x 3hrs)
NumBlocks = length(NumTrialsPerBlock);            % Number of blocks
TotalTrials = sum(NumTrialsPerBlock);             % Total number of trials
PRewardOdor1 = [];                                % Probability of reward for Odor1 in each trial
PRewardOdor2 = [];                                % Probability of reward for Odor2 in each trial

% Create a list of probabilities for each trial
for n = 1:length(NumTrialsPerBlock)
    tmpOdor1 = repmat(Odor1_RewardProbability(n), NumTrialsPerBlock(n), 1);
    tmpOdor2 = repmat(Odor2_RewardProbability(n), NumTrialsPerBlock(n), 1);
    PRewardOdor1 = [PRewardOdor1; tmpOdor1];
    PRewardOdor2 = [PRewardOdor2; tmpOdor2];
    clear tempOdor1 tempOdor2
end

% Test status for all LED controls
olfactoryArena_LED_control(hLEDController, 'RESET');                    % Reset all LEDs
olfactoryArena_LED_control(hLEDController, 'IR', IRIntensity);          % Set IR LED intensity
olfactoryArena_LED_control(hLEDController, 'RED', RedIntensity1);        % Set Red LED intensity (Odor 1)
olfactoryArena_LED_control(hLEDController, 'GREEN', GreenIntensity1);    % Set Green LED intensity (Odor 1)
olfactoryArena_LED_control(hLEDController, 'BLUE', BlueIntensity1);      % Set Blue LED intensity (Odor 1)
olfactoryArena_LED_control(hLEDController, 'RED', RedIntensity2);        % Set Red LED intensity (Odor 2)
olfactoryArena_LED_control(hLEDController, 'GREEN', GreenIntensity2);    % Set Green LED intensity (Odor 2)
olfactoryArena_LED_control(hLEDController, 'BLUE', BlueIntensity2);      % Set Blue LED intensity (Odor 2)

% Define pulse train for Optogenetic Stimulation

param1.pulse_width = PulseWidth1;                 % Width of pulse (in ms)
param1.pulse_period = PulsePeriod1;               % Period of pulse (in ms)
param1.number = PulseNumber1;                     % Number of pulses
param1.off = Off1;                                % Dead time after pulse (in ms)
param1.wait = Wait1;                              % Delay before pulse (in ms)
param1.iterations = Iterations1;                  % Number of times the pulse train is repeated
param1.color = OptoColor1;                        % RED for CSChrimson, GREEN for Chronos
param1.intensity = max([RedIntensity1,BlueIntensity1,GreenIntensity1]); % Stimulation Intensity

param2.pulse_width = PulseWidth2;                 % Width of pulse (in ms)
param2.pulse_period = PulsePeriod2;               % Period of pulse (in ms)
param2.number = PulseNumber2;                     % Number of pulses
param2.off = Off2;                                % Dead time after pulse (in ms)
param2.wait = Wait2;                              % Delay before pulse (in ms)
param2.iterations = Iterations2;                  % Number of times the pulse train is repeated
param2.color = OptoColor2;                        % RED for CSChrimson, GREEN for Chronos
param2.intensity = max([RedIntensity2,BlueIntensity2,GreenIntensity2]); % Stimulation Intensity


%% Initialize the matrices to accumulate data
% Set AirArm = 0 at initiation of experiment and immediately after making a choice (crossing into a reward zone)

% Trial-wise updated variables
AirArm = zeros(1, 4);                           % Which arm has air delivered to it (Home Arm) in each arena
TrialCounter = zeros(1, 4);                     % Which trial is being performed in each arena
ArmRandomizer = NaN(1, 4);                      % Which arm is delivered which odor (Wrt. Home Arm) in each arena
RState_Odor1 = zeros(1, 4);                       % Reward state for Odor1 in each arena
RState_Odor2 = zeros(1, 4);                       % Reward state for Odor2 in each arena

% Frame-wise accumulated variables
AirArmMat = NaN(MaxFrames, 4);                  % Accumulated AirArm
TimeStampMat = NaN(MaxFrames, 1);               % Accumulated TimeStamp
CurrentArmMat = NaN(MaxFrames, 4);              % Accumulated CurrentArm
FlyCentroidsMat = NaN(2, 4, MaxFrames);         % Accumulated Fly position
FlyAreasMat = NaN(MaxFrames, 4);                % Accumulated area of fly blobs
TrialCounterMat = NaN(MaxFrames, 4);          % Accumulated TrialCounter

% Trial-wise accumulated variables
ArmRandomizerMat = NaN(TotalTrials, 4);         % Accumulated ArmRandomizer
FlyChoiceMat = NaN(TotalTrials, 4);             % Accumulated Fly Choices
FlyRewardMat = NaN(TotalTrials, 4);             % Accumulated Fly Rewards
RewardStateTallyOdor1 = NaN(TotalTrials, 4);      % Accumulated Reward State for Odor1
RewardStateTallyOdor2 = NaN(TotalTrials, 4);      % Accumulated Reward State for Odor2

%% ArenaIdx to Arena and Quadrant mapping
PanelA2A = [0,1,2,3];
PanelA2Q = ["0001" "0010" "1000" "0100"];
Odor2Name = ["AIR",Odor1ID, Odor2ID];

%% Begin the video acquisition

% pause(10);                                          % Wait for the finishing all previous memory allocations to prevent crashes
vid = videoinput('mwspinnakerimaq', 1, 'Mono8');    % Initialize video input object
triggerconfig(vid, 'manual');                       % Set the trigger mode to manual
vid.TriggerRepeat = Inf;                            % Allow to indefinitely repeat the trigger signal
vid.FramesPerTrigger = 1;                           % Acquire one frame per trigger

% Start the video aquisition
start(vid);
pause(1);                                           % Wait for the camera to start up

trigger(vid);                                       % Start the acquisition
[lastImage, FrameTime, MetaData] = getdata(vid, 1); % Get the first image
FrameCounter = 1;                                   % Frame counter

for i = 1:MaxFrames

    trigger(vid); % Trigger the camera
    
%     if vid.FramesAvailable > 0
        [thisImage, FrameTime, MetaData] = getdata(vid, 1); % Get the image
%     else
%         continue;
%     end
    
    % Start Image Processing

    % Frame-by-Frame Difference based tracking
    diff_im = lastImage - thisImage; % Calculate the difference between the two images (Optical Flow)
    threshold1 = diff_im / 128 > 0.0; % Threshold the difference image and use unsigned 8-bit integer hack for efficiency
    lastImage = thisImage; % Update the last image

    % Contrast-based tracking
    FrameImage = imcomplement(thisImage); % Invert the image
    SubtractedImage = FrameImage - BackgroundImage; % Subtract the background image
    threshold2 = SubtractedImage / 128 > 0.0; % Threshold the difference image and use unsigned 8-bit integer hack for efficiency
    
    % Consensus tracked image
    CurrentImage = threshold1 | threshold2;

    % Find the centroids and areas of the blobs
    Spots = regionprops(CurrentImage, 'Centroid', 'Area');

    % Filter small blobs due to noise
    Spots = Spots(find([Spots.Area] > 7)); 
    
    if verbose > 1
        disp([num2str(length(Spots)) ' spots found'])
    end
    
    % Make sure atleast one spot is found, otherwise skip this frame
    if isempty(Spots)
        disp('No flies found, skipping frame')
        continue
    end

    % Extract the areas of the blobs
    As = extractfield(Spots, 'Area');

    % Return Arm-Arena location of every blob in LinearArmIdx below:
    % +1 at every BlobsLocs through all 12 arm masks in a stack
    % Return which arms have blobs by searching for values of 1+1=2
    
    % Extract the centroids of the blobs and reshape them into a matrix of column vectors [Xpos Ypos] and get the pixel positions
    Cs = extractfield(Spots, 'Centroid');
    Cs = reshape(Cs, 2, []); 
    BlobLocs = [Cs(2, :)' Cs(1, :)'];
    BlobsLocs = round(BlobLocs);

    % Convert locations to linear indexes to speed up making an image with 1 for every blob
    BlobLinearIdx = sub2ind(size(AllMasks), BlobsLocs(:, 1), BlobsLocs(:, 2));
    BlobsImage = zeros(size(AllMasks));
    % Use linear indexes to insert 1 for every blob
    BlobsImage(BlobLinearIdx) = 1;
    
    if showflies
        se = offsetstrel('ball', 25, 25); 
        dilatedI = imdilate(BlobsImage, se); 
        imagesc(AllMasks.*(dilatedI-24)+sum(RewardMaskStack,3)) % For debugging
        caxis([0 3]);
    end
    
    % Add the blobs image to the Masks to get the valid fly positions
    DetectionStack = MaskStack + BlobsImage;

    % LinearArmIdx are the stack layers with a blob hit
    [~, ~, LinearArmIdx] = ind2sub(size(MaskStack), find(DetectionStack > 1)); 

    % Blank Arena and Arm since previous frame may have had a different number of blobs than current frame
    Arena = [];
    Arm = [];

    % Find the arm and arena for every blob including any duplicates
    ArmMatrix = [1:3; 4:6; 7:9; 10:12]';
    [Arm, Arena] = ind2sub(size(ArmMatrix), LinearArmIdx);
    
    % Return 'Good' Arena indexes where only 1 blob was found
    GoodArenaIdx = find(hist(Arena, 1:4) == 1);
    
    % Return Spot indexes of blobs found in 'Good' Arenas
    GoodBlobIdx = find(ismember(Arena, GoodArenaIdx));

    % Update variables
    CurrentArm = NaN(1, 4);
    CurrentArm(GoodArenaIdx) = Arm(GoodBlobIdx);
    CurrentArmMat(FrameCounter, :) = CurrentArm;
    TimeStampMat(FrameCounter, :) = FrameTime;
    
    Areas = NaN(1, 4);
    Areas(GoodArenaIdx) = As(GoodBlobIdx);

    FlyAreasMat(FrameCounter, :) = Areas;
    FlyCentroidsMat(:, GoodArenaIdx, FrameCounter) = Cs(:, GoodBlobIdx);

    %% If any AirArm values == 0 (at start or after reward recieved) AND a single blob is detected in arm then 
    % 1) flip odor delivery valves to reset home position
    % 2) increment TrialCounter 
    % 3) update AirArm
    % Variables in this loop update Trial-wise
    
    ArenaReady = find(AirArm(GoodArenaIdx) == 0); % Find the good arenas with AirArm == 0
    ArenaEngaged = GoodArenaIdx(ArenaReady);

    if ~isempty(ArenaEngaged) % If any good arenas have been engaged

        for i = randperm(length(ArenaEngaged)) % Randomize the order of the arenas to prevent bias
            
            % If TrialCounter for this arena is equal to the number of trials, flip all arms to air.
            if TrialCounter(ArenaEngaged(i)) >= TotalTrials
                if TrialCounter(ArenaEngaged(i)) == TotalTrials
                    disp(['Arena ' num2str(ArenaEngaged(i)) ' has reached the maximum number of trials'])
                    disp('Flipping all arms to air')
                    TrialCounter(ArenaEngaged(i)) = TrialCounter(ArenaEngaged(i)) + 1;
                    arenaOdorsMsg.arena = uint8(PanelA2A(ArenaEngaged(i)));
                    arenaOdorsMsg.odors = uint8([0 0 0]);
                    send(arenaOdorsPub, arenaOdorsMsg);
                end
                continue;
            end

            AA = AirArm(ArenaEngaged(i)); % Get the AirArm value of the arena
            CA = CurrentArm(ArenaEngaged(i)); % Get the CurrentArm value of the arena

            % To keep AA = 0 if blob not detected in this arm (at experiment start) otherwise AirArm flips to
            % zero only when fly detected crossing reward boundary

            if ~isnan(CA) % If a blob is detected in this arm
                AA = CA;
            else 
                AA = 0; % If no blob detected in this arm
            end

            AR = round(rand); % Randomly choose which side to deliver which odor
            
            % >>MAYBE I SHOULD MAKE AIRARM=3 THE STARTING STATE AND USE THESE ARM INDEXES

            switch AR
                case 0

                    if AA == 1; OdorVec = [0, 1, 2]; 
                    elseif AA == 2; OdorVec = [2, 0, 1];
                    elseif AA == 3; OdorVec = [1, 2, 0];
                    end

                case 1

                    if AA == 1; OdorVec = [0, 2, 1];
                    elseif AA == 2; OdorVec = [1, 0, 2];
                    elseif AA == 3; OdorVec = [2, 1, 0];
                    end

            end
            

            % Flip the odor valves to the side chosen
            arenaOdorsMsg.arena = uint8(PanelA2A(ArenaEngaged(i)));
            arenaOdorsMsg.odors = uint8(OdorVec);
            send(arenaOdorsPub, arenaOdorsMsg);
            
            % Update variables
            TrialCounter(ArenaEngaged(i)) = TrialCounter(ArenaEngaged(i)) + 1; % Increment the TrialCounter for this arena
            
            AirArm(ArenaEngaged(i)) = AA; %If this starts zero and a fly isn't detected in that arena it should stay zero

            ArmRandomizer(ArenaEngaged(i)) = AR;
            ArmRandomizerMat(TrialCounter(ArenaEngaged(i)), ArenaEngaged(i)) = AR;
            
            disp(['(Arena ' num2str(ArenaEngaged(i)) ' Trial ' num2str(TrialCounter(ArenaEngaged(i))) ') Engaged Arm : ' num2str(AA) ' as Home'])
            if verbose > 0
                disp(['(Arena ' num2str(ArenaEngaged(i)) ' Trial ' num2str(TrialCounter(ArenaEngaged(i))) ') Arm 1: ' num2str(Odor2Name(OdorVec(1)+1)) '    Arm 2: ' num2str(Odor2Name(OdorVec(2)+1)) '    Arm 3: ' num2str(Odor2Name(OdorVec(3)+1))])
            end

        end

    end
    
    TrialCounterMat(FrameCounter,:) = TrialCounter;

    AirArmMat(FrameCounter, :) = AirArm;
    
    % When there are less than 4 flies, if all areas that have had 1 or
    % more trials have reached maximum, the acquisition should end.
    % Thus if all active arenas are done, break out of the loop
    if all(TrialCounter(activeArenas) > TotalTrials)
        break;
    end
    
    load('Z:\Rishika\4Y-Maze\Y4ArenaState.mat','earlystop');
    if earlystop
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
        break;
    end
    
    % Determine if a fly is in a reward zones using same method as for detecting arms
    %% FlyChoice = 0 for Odor 1 (Typically OCT) and 1 for Odor 2 (Typically MCH)

    % A FlyChoice is when that reward zone is in a different Arm than AirArm
    % Return FlyChoice - otherwise (i.e. if RewardArena = AirArm) keep empty
    
    RewardDetectionStack = RewardMaskStack + BlobsImage;
    
    % NOTE THERE MAY BE FLIES ALREADY IN A DIFFERENT ARENA REWARDZONE WHEN A FLY NEWLY CROSSES INTO A ZONE
    [~, ~, LinearRewardIdx] = ind2sub(size(RewardMaskStack), find(RewardDetectionStack > 1));
    
    % Reset these values on each run
    RewardArena = [];
    RewardArm = [];
    
    ArmMatrix = [1:3; 4:6; 7:9; 10:12]';
    % Return Arm (1/2/3) & Arena (1/2/3/4) for *every* blob including duplicates
    [RewardArm, RewardArena] = ind2sub(size(ArmMatrix), LinearRewardIdx);
    % Ensure RewardArena can only be an Arena where exactly 1 blob detected
    RewardArena = RewardArena(find(ismember(RewardArena, GoodArenaIdx)));

    % In loop below only get an entry for FlyChoice if the fly is currently
    % in a reward zone that is different from AirArm.  Otherwise FlyChoice is empty
    % Loop increments by Trial
    FlyChoice = [];

    if ~isempty(RewardArena)

        for Idx = randperm(length(RewardArena)) % Randomize the order of the reward zones to prevent bias

            RewardArenaIdx = RewardArena(Idx);
            
            % HERE TRIALCOUNTER IS A SINGLE ROW VECTOR
            TrialCounterIdx = TrialCounter(RewardArenaIdx);

            % If trial counter is more than total number of trials, continue to the next arena
            if TrialCounterIdx > TotalTrials
                continue
            end
            
            AA = AirArm(RewardArenaIdx); % IS THIS CORRECT?
            RA = RewardArm(Idx);
            % CHECK AR is accurte
            AR = ArmRandomizer(RewardArenaIdx);
            
            switch AR
                case 0
                    if AA == 1 && RA == 2 || AA == 2 && RA == 3 || AA == 3 && RA == 1 % Odor 1 (typically Odor1) chosen
                        FlyChoice = 0; % Odor 1 (typically OCT) was chosen
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') Odor 1 (' Odor1ID ') Chosen ie. Arm ' num2str(RA)]) 
                    elseif AA == 1 && RA == 3 || AA == 2 && RA == 1 || AA == 3 && RA == 2 % Odor 2 (typically Odor2) chosen
                        FlyChoice = 1; % Odor 2 (typically MCH) was chosen
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') Odor 2 (' Odor2ID ') Chosen ie. Arm ' num2str(RA)])
                    else % If same arm as AirArm
                        FlyChoice = []; % No fly choice
                    end
                case 1
                    if AA == 1 && RA == 3 || AA == 2 && RA == 1 || AA == 3 && RA == 2 % Odor 1 (typically Odor1) chosen
                        FlyChoice = 0; % Odor 1 (typically OCT) was chosen
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') Odor 1 (' Odor1ID ') Chosen ie. Arm ' num2str(RA)]) 
                    elseif AA == 1 && RA == 2 || AA == 2 && RA == 3 || AA == 3 && RA == 1 % Odor 2 (typically Odor2) chosen
                        FlyChoice = 1; % Odor 2 (typically MCH) was chosen
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') Odor 2 (' Odor2ID ') Chosen ie. Arm ' num2str(RA)])
                    else % If same arm as AirArm
                        FlyChoice = []; % No fly choice
                    end
            end

            % Give Reward if FlyChoice is not empty
            if ~isempty(FlyChoice)
                AirArm(RewardArenaIdx) = 0;
                
                % Update variables
                FlyChoiceMat(TrialCounter(RewardArenaIdx),RewardArenaIdx) = FlyChoice ;
                FlyRewardMat(TrialCounter(RewardArenaIdx),RewardArenaIdx) = 0; % Default value is set to zero but changed to 1 if fly is rewarded
                
                % Get Reward probabilities set by user input
                Pr_Odor1 = PRewardOdor1(TrialCounterIdx);
                Pr_Odor2 = PRewardOdor2(TrialCounterIdx);
                
                % Get Reward State initialized at zero for each arena but updated below
                RS_Odor1 = RState_Odor1(RewardArenaIdx);
                RS_Odor2 = RState_Odor2(RewardArenaIdx);
                
                Randomizer_Odor1 = rand;
                Randomizer_Odor2 = rand;

                if RS_Odor1 == 1 && baited
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor1ID 'Baited']);
                    end
                elseif Pr_Odor1 >= Randomizer_Odor1 
                    RS_Odor1 = 1; 
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor1ID 'Drawn']);
                    end
                else 
                    RS_Odor1 = 0;
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor1ID 'Empty']);
                    end
                end

                if RS_Odor2 == 1 && baited
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor2ID 'Baited']);
                    end
                elseif Pr_Odor2 >= Randomizer_Odor2
                    RS_Odor2 = 1; 
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor2ID 'Drawn']);
                    end
                else 
                    RS_Odor2 = 0;
                    if verbose > 1
                        disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor2ID 'Empty']);
                    end
                end

                if verbose > 0
                    disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor1ID ' Reward State: ' num2str(RS_Odor1) '   ' Odor2ID ' Reward State: ' num2str(RS_Odor2)]);
                end

                % Accumulate RStates here so we know what was PRESENTED each trial
                % If reward is drawn and chosen on a given trial the tally
                % sequence will be  0  1(drawn) 0(chosen)
                % If reward is not chosen and stays baited the tally
                % sequence will be 0 1(drawn) 1(baited) until it goes 0(chosen)

                RState_Odor1(RewardArenaIdx) = RS_Odor1;
                RState_Odor2(RewardArenaIdx) = RS_Odor2;

                RewardStateTallyOdor1(TrialCounterIdx, RewardArenaIdx) = RS_Odor1;
                RewardStateTallyOdor2(TrialCounterIdx, RewardArenaIdx) = RS_Odor2;
                
                param1.quadrants = convertStringsToChars(PanelA2Q(RewardArenaIdx));
                param1.panel = 1;
                param2.quadrants = convertStringsToChars(PanelA2Q(RewardArenaIdx));
                param2.panel = 1;

                if FlyChoice == 0 && RS_Odor1 == 1
                    RState_Odor1(RewardArenaIdx) = 0; % Update RewardState only inside this 'if' to maintain baiting
                    
                    FlyRewardMat(TrialCounter(RewardArenaIdx),RewardArenaIdx) = 1;
                    
                    % Send pulse train information to the LED controller
                    olfactoryArena_LED_control(hComm.hLEDController, 'PULSE', param1, 0);
                    olfactoryArena_LED_control(hLEDController, 'RED', 0, 0);
                    olfactoryArena_LED_control(hLEDController, 'GREEN', 0, 0);
                    olfactoryArena_LED_control(hLEDController, 'BLUE', 0, 0);
                    
                    if RedIntensity1 > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'RED_QUAD', param1, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    elseif BlueIntensity1 > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'BLUE_QUAD', param1, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    elseif GreenIntensity > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'GREEN_QUAD', param1, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    end
                    
                    disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor1ID ' chosen and stimulated']);
                end

                % Same steps for Odor2 reward
                if FlyChoice == 1 && RS_Odor2 == 1
                    RState_Odor2(RewardArenaIdx) = 0; % Update RewardState only inside this 'if' to maintain baiting
                    
                    FlyRewardMat(TrialCounter(RewardArenaIdx),RewardArenaIdx) = 1;

                    % Send pulse train information to the LED controller
                    olfactoryArena_LED_control(hComm.hLEDController, 'PULSE', param2, 0);
                    olfactoryArena_LED_control(hLEDController, 'RED', 0, 0);
                    olfactoryArena_LED_control(hLEDController, 'GREEN', 0, 0);
                    olfactoryArena_LED_control(hLEDController, 'BLUE', 0, 0);
                    
                    if RedIntensity2 > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'RED_QUAD', param2, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    elseif BlueIntensity2 > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'BLUE_QUAD', param2, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    elseif GreenIntensity2 > 0
                        olfactoryArena_LED_control(hComm.hLEDController, 'GREEN_QUAD', param2, 0);
                        olfactoryArena_LED_control(hComm.hLEDController, 'RUN', 0, 0);
                    end
                    disp(['(Arena ' num2str(RewardArenaIdx) ' Trial ' num2str(TrialCounterIdx) ') ' Odor2ID ' chosen and stimulated']);
                end

            end

        end

    end

    FrameCounter = FrameCounter + 1; % Increment the frame counter
end

stop(vid);
flushdata(vid)
delete(vid)
clear vid


% Store all important information in the YArenaInfo struct

% Settings
YArenaInfo.rigName = rigName;
YArenaInfo.ActiveArenas = activeArenas;
YArenaInfo.ExperimentName = expname;
YArenaInfo.ExperimentDateTime = exptime;
YArenaInfo.Fly1 = fly1genotype;
YArenaInfo.Fly2 = fly2genotype;
YArenaInfo.Fly3 = fly3genotype;
YArenaInfo.Fly4 = fly4genotype;
YArenaInfo.IRIntensity = IRIntensity;
YArenaInfo.RedIntensity1 = RedIntensity1;
YArenaInfo.GreenIntensity1 = GreenIntensity1;
YArenaInfo.BlueIntensity1 = BlueIntensity1;
YArenaInfo.PulseWidth1 = PulseWidth1;
YArenaInfo.PulsePeriod1 = PulsePeriod1;
YArenaInfo.PulseNumber1 = PulseNumber1;
YArenaInfo.Wait1 = Wait1;
YArenaInfo.Off1 = Off1;
YArenaInfo.Iterations1 = Iterations1;
YArenaInfo.OptoColor1 = OptoColor1;
YArenaInfo.RedIntensity2 = RedIntensity2;
YArenaInfo.GreenIntensity2 = GreenIntensity2;
YArenaInfo.BlueIntensity2 = BlueIntensity2;
YArenaInfo.PulseWidth2 = PulseWidth2;
YArenaInfo.PulsePeriod2 = PulsePeriod2;
YArenaInfo.PulseNumber2 = PulseNumber2;
YArenaInfo.Wait2 = Wait2;
YArenaInfo.Off2 = Off2;
YArenaInfo.Iterations2 = Iterations2;
YArenaInfo.OptoColor2 = OptoColor2;
YArenaInfo.MaskStack = MaskStack;
YArenaInfo.RewardMaskStack = RewardMaskStack;
YArenaInfo.Odor1ID = Odor1ID;
YArenaInfo.Odor2ID = Odor2ID;
YArenaInfo.NumTrialsPerBlock = NumTrialsPerBlock;
YArenaInfo.NumBlocks = NumBlocks;
YArenaInfo.TotalTrials = TotalTrials;
YArenaInfo.Odor1_RewardProbability = Odor1_RewardProbability;
YArenaInfo.Odor2_RewardProbability = Odor2_RewardProbability;
YArenaInfo.baited = baited;
YArenaInfo.FrameCount = FrameCounter;
YArenaInfo.SaveDir = savedir;
YArenaInfo.MaskFile = allMaskDirectory;
YArenaInfo.AddlComments = addlcomments;

% Background Image information
YArenaInfo.BkgdMovieLength = BkgdMovieLength;
YArenaInfo.BackgroundImage = BackgroundImage;

% Accumulated Variables
YArenaInfo.AirArmMatrix = AirArmMat;
YArenaInfo.TimeStampMatrix = TimeStampMat;
YArenaInfo.CurrentArmMatrix = CurrentArmMat;
YArenaInfo.FlyCentroidsMatrix = FlyCentroidsMat;
YArenaInfo.FlyAreasMatrix = FlyAreasMat;
YArenaInfo.ArmRandomizerMatrix = ArmRandomizerMat;
YArenaInfo.TrialCounterMatrix = TrialCounterMat;
YArenaInfo.FlyChoiceMatrix = FlyChoiceMat;
YArenaInfo.FlyRewardMatrix = FlyRewardMat;
YArenaInfo.RewardStateTallyOdor1 = RewardStateTallyOdor1;
YArenaInfo.RewardStateTallyOdor2 = RewardStateTallyOdor2;

mkdir([savedir expname '-' exptime '/']);
save([savedir expname '-' exptime '/' 'YArenaInfo.mat'], 'YArenaInfo');