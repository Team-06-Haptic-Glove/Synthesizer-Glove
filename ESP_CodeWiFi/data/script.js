//-----Lotta testing shit-----//
    //
    setInterval(function(){
        var xttp = new XMLHttpRequest();
        xttp.onreadystatechange = function(){
            if(this.readyState == 4 && this.status == 200){
                console.log(this.responseText);
            }
        };
        xhttp.open("GET", "/sensors", true);
        xhttp.send();
    }, 30000);

//Declaring aspect variables
    var volume;
        var maxVol;
        var minVol;
    var hertz;
        var maxHertz;
        var minHertz;
    var bpmtest;
    var bpmMax;
    var bpmMin;
    //Declare additional variables
    var hertzTest; //To be used in creating octaves from frequency
    var setValMax; //Max value decided for input sliders, range from setVal -> setValMax
    //Declare variable arrays
    var trackName; //Array of track names
    var play;      //Array for whether current track is active/low
    var edit;      //Array to determine which track is currently being altered by sensor inputs
    var playVol;   //Array of all the volume tracks
        var playVolInner;

//------Functions-------//
    //loadMe() On page load, initializes all variables
    function loadMe(){
        showingSet = false;
        showingRef = false;
        volume = -15;
        maxVol = 20;
        minVol = -15;
        hertz = 0;
        maxHertz = 1000;
        minHertz = 0;
        bpmMax = 240;
        bpmMin = 30;
        play = [false, false, false, false, false, false, false, false, false, false];
        m_play = false
        edit = [true, false, false, false, false, false, false, false, false, false];
        trackName = ["Buffer", "Kick", "Snare", "Synth 1", "Synth 2", "Synth 3", "Synth 4", "Synth 5", "Osc 1", "Osc 2"];
        playVol = ["", "playVol1","playVol2","playVol3","playVol4","playVol5","playVol6","playVol7","playVol8","playVol9",]
        playVolInner = ["", "playVolInner1","playVolInner2","playVolInner3","playVolInner4","playVolInner5","playVolInner6","playVolInner7","playVolInner8","playVolInner9",]
        loop = [0,0,0,0,0,0,0,0,0,0];
        track = [0,0,0,0,0,0,0,0,0,0];
        freq = ["C1", "C1", "C1", "C1", "C1", "C1", "C1", "C1", "C1", "C1"]
        bpm = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        hertzTest = 0;
        setValMax = 100;
        currentTrack = 0;

        //Turning buffer/stop light on when site is loaded
        document.getElementById("playLight0").style.backgroundColor = "#2DD201";
        document.getElementById("stop").style.backgroundColor = "red";


        //Backtrack initilization
            track[1] = new Tone.MembraneSynth().toDestination();
            track[1].volume.value = -15;

            track[2] = new Tone.NoiseSynth().toDestination();
            track[2].volume.value = -15;
        //Synthtime
            
            for (i = 3;i < 8;i++){
                track[i] = new Tone.Synth().toDestination();
                track[i].volume.value = -15;
            }

        //Initialize new oscillator, done once
            track[8] = new Tone.Oscillator(freq[8],"sine").toDestination();
            track[8].volume.value = -15;

            track[9] = new Tone.Oscillator(freq[9],"square").toDestination();
            track[9].volume.value = -15;
    }

    //openSettings() Opens up the setting menu
    function openSettings(){
        document.getElementById("references").style.display = "none";
        if (showingSet == false){
            document.getElementById("settings").style.display = "block";
            showingSet = true;
        }
        else if (showingSet == true){
            document.getElementById("settings").style.display = "none";
            showingSet = false;
        }
    }
    //openRefs() Opens up additional references menu
    function openRefs(){
        document.getElementById("settings").style.display = "none";
        if (showingRef == false){
            document.getElementById("references").style.display = "block";
            showingRef = true;
        }
        else if (showingRef == true){
            document.getElementById("references").style.display = "none";
            showingRef = false;
        }
    }

    //setFlex(Finger, Value) Takes in variable value and displays numerically/using bar
    //Finger: The finger which is affecting the bar
    //Value: The variable input created by the flex sensors
    function setFlex(finger, value){
        //Create temporary local variable to hold the slider value
        var setVal = value;

        if (finger == thumb){
            //Adjusts raw volume and set as the actual volume
                volume = ( Number(minVol) + ( (Number(maxVol)-Number(minVol))*(setVal/100) ) );
                volume = Number(volume.toFixed(1));
            //Checks for previous button to be correct one, prevents editing of all tracks
                for (i = 0; i < edit.length; i++){
                    if (edit[i] == true){
                        track[i].volume.value = volume;
                        var volDivInner = playVolInner[i];
                        var volDiv = playVol[i];
                        document.getElementById(volDivInner).style.height = (100-setVal) + "%";
                        if (setVal < 30)
                            document.getElementById(volDiv).style.backgroundColor = '#0AA30C';
                        else if (setVal > 40 && setVal < 90)
                            document.getElementById(volDiv).style.backgroundColor = '#D0D40F';
                        else if (setVal > 90)
                            document.getElementById(volDiv).style.backgroundColor = '#E12120';
                    }
                }
            //Update the UI
            document.getElementById("pointerBar").style.width = setVal + "%";
            document.getElementById("flexNum1").innerHTML = volume + "db";
            if (setVal < 30)
                document.getElementById("pointerBar").style.backgroundColor = '#0AA30C';
            else if (setVal > 40 && setVal < 90)
                document.getElementById("pointerBar").style.backgroundColor = '#D0D40F';
            else if (setVal > 90)
                document.getElementById("pointerBar").style.backgroundColor = '#E12120';
        }
        if (finger == pointer){
            //Adjusts Hz of the output oscillator
                //Declare min/max in hertz
                hertzTest = ((12)*(setVal/setValMax))+28;
                hertz = (2**((hertzTest.toFixed(0)-49)/12))*440
                hertz = hertz.toFixed(2);
            for (i = 3; i < edit.length; i++){
                if (edit[i] == true)
                    freq[i] = hertz;
            }
            for (i = 8; i < edit.length; i++){
                if (edit[i] == true)
                    track[i].frequency.value = hertz;
            }
            //Update the UI
            document.getElementById("middleBar").style.width = setVal + "%";
            document.getElementById("flexNum2").innerHTML = hertz + "hz";
            if (setVal < 30)
                document.getElementById("middleBar").style.backgroundColor = '#0AA30C';
            else if (setVal > 40 && setVal < 90)
                document.getElementById("middleBar").style.backgroundColor = '#D0D40F';
            else if (setVal > 90)
                document.getElementById("middleBar").style.backgroundColor = '#E12120';

        }
        if (finger == middle){
            //Adjust 1-100 range to bpm with range of 0-1 with increments of 0.1
            var tempBPM = 60 / (((Number(bpmMax)-Number(bpmMin))*(setVal/100)) + Number(bpmMin));
            tempBPM = (Math.round((tempBPM*100)/5)*5)/100;
            for (i = 0; i < 8; i++){
                if (edit[i] == true)
                    loop[i].interval = (tempBPM);
            }
            //Update UI
            document.getElementById("thumbBar").style.width = setVal + "%";
            document.getElementById("flexNum3").innerHTML = (60 / (Number(tempBPM))).toFixed(0) + " BPM";
            if (setVal < 30)
                document.getElementById("thumbBar").style.backgroundColor = '#0AA30C';
            else if (setVal > 40 && setVal < 90)
                document.getElementById("thumbBar").style.backgroundColor = '#D0D40F';
            else if (setVal > 90)
                document.getElementById("thumbBar").style.backgroundColor = '#E12120';
        }

        //Clear and display updated values to console
        console.clear();
        console.log("Volume set to: " + volume + "db");
        console.log("Pitch set to: " + hertz + "hz");
        console.log("BPM set to: " + 60 / (Number(tempBPM)) + "BPM");
        console.log(tempBPM);
    }

    //playSound() Acts as the global enable play/stop
    function playSound(){
        Tone.start();
        if (m_play == false){
            m_play = true;
            Tone.Transport.start();
            document.getElementById("play").style.backgroundColor = "green";
            document.getElementById("stop").style.backgroundColor = "grey";
            if (play[8] == true)
                track[8].start();
            if (play[9] == true)
                track[9].start();
        }
        else if (m_play == true){
            m_play = false;
            Tone.Transport.stop();
            document.getElementById("stop").style.backgroundColor = "red";
            document.getElementById("play").style.backgroundColor = "grey";
            track[8].stop();
            track[9].stop();
        }
    }

    //changeTrack() Cycles to next / previous? track on button press
    //Link changeTrack to swipe gesture from glove
    function changeTrack(){
        Tone.start();
        console.log(currentTrack);
        currentTrack++;
        if(currentTrack > 9){
            currentTrack = 0;
        }
        /*
        if (direction == 'next'){
            currentTrack++;
            if(currentTrack > 9){
                currentTrack = 0;
            }
        }
        else if (direction == 'last'){
            currentTrack--;
            if (currentTrack < 0){
                currentTrack = 9;
            }
        }
        */
        edit = [false, false, false, false, false, false, false, false, false, false]
        edit[currentTrack] = true;
        document.getElementById("trackDisplay").innerHTML = trackName[currentTrack];

        if (edit[0] == true)
            //buffer light on
            document.getElementById("playLight0").style.backgroundColor = "#2DD201";
        else if (edit[0] == false)
            //buffer light off
            document.getElementById("playLight0").style.backgroundColor = "grey";
    }

    //playTrack Plays/Stops the track which is currently selected
    function playTrack(){
        Tone.start();
            //Kick
            if (edit[1] == true){            
                if (play[1] == false) {
                    play[1] = true;
                    //kick light on
                    document.getElementById("playLight1").style.backgroundColor = "#2DD201";
                    //looping the sound
                    loop[1] = new Tone.Loop(time => {
                        track[1].triggerAttackRelease("C1", "8n", time);
                    }, 0.5).start(0); 
                } 
                else if (play[1] == true){
                    play[1] = false;
                    document.getElementById("playLight1").style.backgroundColor = "grey";
                    loop[1].stop();
                }
            }
            //Snare
            else if (edit[2] == true){
                if (play[2] == false) {
                    play[2] = true;
                    //snare light on
                    document.getElementById("playLight2").style.backgroundColor = "#2DD201";
                    loop[2] = new Tone.Loop(time =>{
                        track[2].triggerAttackRelease("4n",time)
                    },"1n").start(0);
                } 
                else if (play[2] == true){
                    play[2] = false;
                    document.getElementById("playLight2").style.backgroundColor = "grey";
                    loop[2].stop();
                }
            }
            //Synth 1
            else if (edit[3] == true){                
                if (play[3] == false) {
                    play[3] = true;
                    //synth1 light on
                    document.getElementById("playLight3").style.backgroundColor = "#2DD201";
                    loop[3] = new Tone.Loop(time =>{
                        track[3].triggerAttackRelease(freq[3], "8n", time)
                    },"1n").start(0);
                } 
                else if (play[3] == true){
                    play[3] = false;
                    document.getElementById("playLight3").style.backgroundColor = "grey";
                    loop[3].stop();
                }
            }
            //Synth 2
            else if (edit[4] == true){                
                if (play[4] == false) {
                    play[4] = true;
                    //synth2 light on
                    document.getElementById("playLight4").style.backgroundColor = "#2DD201";
                    loop[4] = new Tone.Loop(time =>{
                        track[4].triggerAttackRelease(freq[4], "8n", time)
                    },"1n").start(0);
                } 
                else if (play[4] == true){
                    play[4] = false;
                    document.getElementById("playLight4").style.backgroundColor = "grey";
                    loop[4].stop();
                }
            }
            //Synth 3
            else if (edit[5] == true){                
                if (play[5] == false) {
                    play[5] = true;
                    //synth3 light on
                    document.getElementById("playLight5").style.backgroundColor = "#2DD201";
                    loop[5] = new Tone.Loop(time =>{
                        track[5].triggerAttackRelease(freq[5], "8n", time)
                    },"1n").start(0);
                } 
                else if (play[5] == true){
                    play[5] = false;
                    document.getElementById("playLight5").style.backgroundColor = "grey";
                    loop[5].stop();
                }
            }
            //Synth 4
            else if (edit[6] == true){                
                if (play[6] == false) {
                    play[6] = true;
                    //synth4 light on
                    document.getElementById("playLight6").style.backgroundColor = "#2DD201";
                    loop[6] = new Tone.Loop(time =>{
                        track[6].triggerAttackRelease(freq[6], "8n", time)
                    },"1n").start(0);
                } 
                else if (play[6] == true){
                    play[6] = false;
                    document.getElementById("playLight6").style.backgroundColor = "grey";
                    loop[6].stop();
                }
            }
            //Synth 5
            else if (edit[7] == true){                
                if (play[7] == false) {
                    play[7] = true;
                    //synth5 light on
                    document.getElementById("playLight7").style.backgroundColor = "#2DD201";
                    loop[7] = new Tone.Loop(time =>{
                        track[7].triggerAttackRelease(freq[7], "8n", time)
                    },"1n").start(0);
                } 
                else if (play[7] == true){
                    play[7] = false;
                    document.getElementById("playLight7").style.backgroundColor = "grey";
                    loop[7].stop();
                }
            }
            //Osc 1
            else if (edit[8] == true){                
                if (play[8] == false) {
                    play[8] = true;
                    //osc1 light on
                    document.getElementById("playLight8").style.backgroundColor = "#2DD201";
                    if (m_play == true)
                        track[8].start();
                } 
                else if (play[8] == true){
                    play[8] = false;
                    document.getElementById("playLight8").style.backgroundColor = "grey";
                    track[8].stop();
                }

            }
            //Osc 1
            else if (edit[9] == true){                
                if (play[9] == false) {
                    play[9] = true;
                    //osc2 light on
                    document.getElementById("playLight9").style.backgroundColor = "#2DD201";
                    if (m_play == true)
                        track[9].start();
                } 
                else if (play[9] == true){
                    play[9] = false;
                    document.getElementById("playLight9").style.backgroundColor = "grey";
                    track[9].stop();
                }
            }
        }

    //setMax/MinVol Allows user to set maximum/minimum volume in decibels using the textbox
    function setMaxVol(volume){ maxVol = volume; console.log(maxVol)}
    function setMinVol(volume){ minVol = volume; console.log(minVol)}
    //setMax/MinBPM Allows user to set maximum/minimum BPM using the text box
    function setMaxBPM(beats){ bpmMax = beats; console.log(bpmMax)}
    function setMinBPM(beats){ bpmMin = beats; console.log(bpmMin)}

    //Who knows?
    function playMe(clip){
        var hiddenAudio = [
            new Audio('./zAssets/Audio/helpmereremix.mp3'),
            new Audio('./zAssets/Audio/VineBoom.mp3'),
            new Audio('./zAssets/Audio/helpmerereremix.mp3'),
            new Audio('./zAssets/Audio/Baba.mp3')
        ];
        hiddenAudio[clip].play();
    }

    function about(name){
        if (name == 1){
            document.getElementById("about").style.backgroundColor = ""
            document.getElementById("aboutGlove").style.display = "block";
            document.getElementById("aboutTeam").style.display = "none";
        }
        if (name == 2){
            document.getElementById("aboutGlove").style.display = "none";
            document.getElementById("aboutTeam").style.display = "block";
        }
    }