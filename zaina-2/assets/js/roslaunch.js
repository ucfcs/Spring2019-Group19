var ros = new ROSLIB.Ros(
{
  url: 'ws://localhost:9090'
});


var txt_listener = new ROSLIB.Topic(
{
 ros: ros,
 name: '/txt_msg',
 messageType: 'std_msgs/String'
});

txt_listener.subscribe(function (m)
{
 document.getElementById("msg").innerHTML = m.data;
 move(1, 0);
});

cmd_vel_listener = new ROSLIB.Topic(
{
 ros: ros,
 name: "/cmd_vel",
 messageType: 'geometry_msgs/Twist'
});

move = function (linear, angular) //bottomSpeed, topSpeed)
{
 var twist = new ROSLIB.Message(
 {
   linear: {
     x: linear,
     y: 0, //bottomSpeed,
     z: 0
   },
   angular: {
     x: 0,
     y: 0, //topSpeed,
     z: angular
   }
 });

 cmd_vel_listener.publish(twist);
}
