bool driveToPoint(){
    //TODO check if the path is blocked and push the robot's location after it turns
    
    if(goForward)
        //start the motors
    else
        //start motors in reverse
    
    double minDist = 2;
    //distance is a function that needs to be called from the Odroid
    while(distanceToTarget > minDist){
        Delay(50);
    }
    
    
    //stop the motors
    
    return true
}

void turnToAngle(){
    //turn the robot by difference degrees
}

//put the following stuff inside main

//wait for the Odroid to start finding walls
while(!goForward){
    Delay(100);
}

//We'll make this end at some point, but that can come after the demo
while(true){
    turnToAngle();
    driveToPoint();
}
