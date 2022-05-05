var display = document.getElementById('imgDiv');
var sound;

const source = new EventSource('/next-facial-expression')
source.onopen = (e) => console.log('Connection requested')
source.onmessage = (msg) => {
    try {
        let data = JSON.parse(msg.data)
        if (data.display) {
            // Update gifs/pics
            display.innerHTML = '<img src=\'static/display/' + data.display + '\'>'
        }
        // Stop previous Sound            
        if (sound) {
            sound.pause();
            sound.currentTime = 0;
        }
        // Load and start new sound
        if (data.audio) {
            let url = 'static/audio/' + data.audio;
            sound = new Audio(url);
            sound.play();
        }
        if(data.info) {
            console.log(data.info)
        }
    } catch (error) {
        console.error('Wrong message type', error);
    }
}

// Stopp audio and close sse connection on reload
window.onbeforeunload = (e) => {
    if (sound) {
        sound.pause();
        sound.currentTime = 0;
    }
    if (source.readyState == source.OPEN) {
        source.close();
    }
}

source.onerror = (error) => {
    console.error("connection disrupted", error);
}
