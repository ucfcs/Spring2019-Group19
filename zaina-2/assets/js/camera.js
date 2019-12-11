var video = document.getElementById('videoElement');
var video2 = document.getElementById('videoElementDos');
var canvas = document.getElementById('canvas'); // create a canvas
var context = canvas.getContext('2d');
var canvas2 = document.getElementById('canvas2'); // create a canvas
var context2 = canvas2.getContext('2d');

this.startstops = 'stop';

this.toggleStartStop = () => {
		let text = 'Stop';
		if (this.startstop == 'stop'){
			this.startstop = 'start';
			navigator.mediaDevices.getUserMedia({ video: true }).then(stream => {
			  video.srcObject = stream; // don't use createObjectURL(MediaStream)
			  return video.play(); // returns a Promise
			})
			.then(()=>{ // enable the button
			  var btn = document.getElementById('camera--trigger');
			  btn.disabled = false;
			  btn.onclick = e => {
			    download()
				//	downloadDos()
			  };
			})
	/*		navigator.mediaDevices.getUserMedia({video:{
    	deviceId: { exact: '0ae05783abe9a4c88167fb04bac674a1d5835932a406ab612f15cc3edb014ca5' }
	}}) // request cam
			.then(stream => {
				video2.srcObject = stream; // don't use createObjectURL(MediaStream)
				return video2.play(); // returns a Promise
			})*/
		}
	else {
			video.pause();
			video2.pause();
		    text = 'Start';
		    this.startstop = 'stop';

		}
		this.startstopbutton.textContent = text;
}

function download(){
  // uses the <a download> to download a Blob
	context.drawImage(video,0,0, canvas.width, canvas.height);
  let a = document.createElement('a');
  a.href = canvas.toDataURL('image/jpeg');
  a.download = 'screenshot.jpg';
  document.body.appendChild(a);
  a.click();
}

function downloadDos(){
  // uses the <a download> to download a Blob
	context2.drawImage(video2,0,0, canvas2.width, canvas2.height);
  let a = document.createElement('a');
  a.href = canvas2.toDataURL('image/jpeg');
  a.download = 'screenshot.jpg';
  document.body.appendChild(a);
  a.click();
}
