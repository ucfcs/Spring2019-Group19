function interactiveKeys() {

   this.keys = {
       KeyI: document.getElementsByClassName("up")[0],
       Comma: document.getElementsByClassName("down")[0],
       KeyJ: document.getElementsByClassName("left")[0],
       KeyL: document.getElementsByClassName("right")[0]
   }


   this.button = document.getElementById('toggle');
   this.startstopbutton = document.getElementById('startstop');


/*   this.mode = 'auto';

   this.toggleModes = () => {


       if ( this.mode == 'auto' ) {

           this.mode = 'manual';
       }

       else {

             text = 'Manual Mode Enabled';
             if (this.mode == 'manual') {
               alert("Please click stop");
             }
       }
       this.button.textContent = text;

   };*/


   this.handleKeys = ( e ) => {

       if ( this.mode == 'auto' ) return;

       let target = this.keys.up;

       switch (e.keyCode) {
           case 74:
               // Left Key Pressed
               target = this.keys.KeyJ;
               move(-1, 0);
               break;

           case 76:
               // Right Key Pressed
               target = this.keys.KeyL;
               move(1, 0);
               break;

           case 73:
               // Up Key Pressed
               target = this.keys.KeyI;
               move(1,1);
               break;

           case 188:
               // Down Key Pressed
               target = keys.Comma;
               move(-1, 1);
               break;
               //needs to be done
               case 85:
                   // retract actuators
                   target = keys.keys.KeyU;
                   move(-1, 0);
                   break;

              case 79:
             // extends actuators
                target = keys.keys.KeyO;
                move(-1, 0);
                break;

            case 77:
                // move hex motors up
                target = keys.keys.KeyM;
                move(-1, 0);
                break;
          case 46:
            // move hex motors down
            target = keys.period;
            move(-1, 0);
            break;
          case 75:
            target =this.keys.KeyK
            move(0,0);
            break;
       }

       if ( e.type == 'keydown' ) {
           target.classList.add("active");
       }

       else {
           target.classList.remove("active");
       }

   };

   this.listen = () => {

       window.addEventListener('keydown', this.handleKeys);
       window.addEventListener('keyup', this.handleKeys);
    //   this.button.addEventListener('click', this.toggleModes);
       this.startstopbutton.addEventListener('click', this.toggleStartStop);

   }

   this.listen();

}

function init() {

   const keys = interactiveKeys();

}

window.addEventListener("load", init);
