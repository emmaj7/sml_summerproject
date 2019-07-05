
// Code for the simulation window.
// Created by: Mikael Glamheden
// 2019-06-24
//
// Edited by: Emma Johansson & Mikael Glamheden
// Last edited: YYYY-MM-DD


let type = "WebGL";
if(!PIXI.utils.isWebGLSupported()){
  type = "canvas"
}
PIXI.utils.sayHello(type); // prints to console if everything is working.

// Aliases
let Application = PIXI.Application,
loader = PIXI.loader,
resources = PIXI.loader.resources,
TextureCache = PIXI.utils.TextureCache,
Sprite = PIXI.Sprite,
Rectangle = PIXI.Rectangle;

// Create a Pixi Application
let app = new Application({
  width: 800,         // default: 800
  height: 800,        // default: 600
  antialias: true,    // default: false
  transparent: false, // default: false
  resolution: 1,      // default: 1
  forceCanvas: false  // force to use drawng API instead of WebGL
});

// Change background color
app.renderer.backgroundColor = 0x061639;

// Access style of window
app.renderer.view.style.position = "absolute";
app.renderer.view.style.left = '800px';
app.renderer.view.style.top = '100px';

// Add the canvas that Pixi automatically created for you to the HTML document
document.body.appendChild(app.view);

// Variables for the simulation
let car;
let PI = Math.PI;
let x_offset = 0.2;
let y_offset = 0.15;
let real_dims = {height: 3, width: 3}; // dimensions of window in meters.
let scale_factor = app.renderer.view.width/real_dims.height;
let last_yaw = 0;
let dt = 1/30; // 1 over 30 Hz
let max = 1.5; // moves (0,0) to center of screen with (1.5, 1.5) being the upper left corner.
let start = {'x': -max + 0.15 ,'y': -max + 0.15 , 'yaw': 0};
let x = start.x;
let y = start.y;
let yaw = start.yaw;
let marg = 0.1;
let text0, text1, text2, text3, text4; // holds the 5 car sprites.
let curr_sprite;
let hit = false; // for collision detection.
let obstacles = [];
//load an image and run the `setup` function when it's done
loader
.add(['./images/svea_car.png',
'./images/svea_cars2.png',
'./images/mars-background2.png',
'./images/crater.png',
'./images/base-v2.png'])
.load(setup);
// This `setup` function will run when the image has loaded
function resize_image(image){
  let imageRatio = image.width / image.height;
  let containerRatio = app.renderer.view.width / app.renderer.view.height;

  if (containerRatio > imageRatio) {
    image.height = image.height / (image.width / app.renderer.view.width);
    image.width = app.renderer.view.width;
    image.position.x = 0;
    image.position.y = (app.renderer.view.height - image.height) / 2;
  } else {
    image.width = image.width / (image.height / app.renderer.view.height);
    image.height = app.renderer.view.height;
    image.position.y = 0;
    image.position.x = (app.renderer.view.width - image.width) / 2;
  }
  return image;
}
function setup() {
  // Create the textures
  let rectangle0 = new Rectangle(0, 288*0, 413, 288);
  text0 = new PIXI.Texture(resources["./images/svea_cars2.png"].texture, rectangle0);
  let rectangle1 = new Rectangle(0, 288*1, 413, 288);
  text1 = new PIXI.Texture(resources["./images/svea_cars2.png"].texture, rectangle1);
  let rectangle2 = new Rectangle(0, 288*2, 413, 288);
  text2 = new PIXI.Texture(resources["./images/svea_cars2.png"].texture, rectangle2);
  let rectangle3 = new Rectangle(0, 288*3, 413, 288);
  text3 = new PIXI.Texture(resources["./images/svea_cars2.png"].texture, rectangle3);
  let rectangle4 = new Rectangle(0, 288*4, 413, 288);
  text4 = new PIXI.Texture(resources["./images/svea_cars2.png"].texture, rectangle4);
  // Create the sprites
  car = new Sprite(text2);
  base = new Sprite(resources["./images/base-v2.png"].texture);

  // initialize background
  background = new Sprite(resources["./images/mars-background2.png"].texture);
  background = resize_image(background);
  app.stage.addChild(background);

  // place obstacles
  for (var i = 0; i < obstacle_list.length; i++) {
    obstacle = new Sprite(resources["./images/crater.png"].texture);
    obstacle.scale.x = 1.5; obstacle.scale.y = 1.5;
    obstaclePx = coordinate_transform(obstacle_list[i].x,
      obstacle_list[i].y,
      obstacle_list[i].yaw);
      obstacle.position.set(obstaclePx.x, obstaclePx.y);
      obstacle.anchor.x = 0.5; obstacle.anchor.y = 0.5;
      app.stage.addChild(obstacle);
      obstacles.push(obstacle);
    }
    // Initilize base
    base.anchor.x = 0.5; base.anchor.y = 0.8;
    base.scale.x = 0.4; base.scale.y = 0.4;
    goalPx = coordinate_transform(goal.x, goal.y, goal.yaw);
    base.position.set(goalPx.x,goalPx.y);
    app.stage.addChild(base);

    // Initialize car
    car.scale.x = 3*(90/app.renderer.width); car.scale.y = 3*(90/app.renderer.height);
    car.anchor.x = 0.5; car.anchor.y = 0.5;
    startPx = coordinate_transform(start.x, start.y, start.yaw);
    car.position.set(startPx.x,startPx.y);
    car.rotation = start.yaw;
    app.stage.addChild(car);
    curr_sprite = 2;
  }
  // transforms between "true" coordinate system and browser window coordinates
  function coordinate_transform(x, y, yaw){
    let coords = {};
    coords.x = scale_factor*(x + max);
    coords.y = app.renderer.view.height - scale_factor*(y + max); // y = 0 is upper corner
    coords.yaw = -yaw;
    return coords;
  }
  // swithc wich sprite to display.
  function change_sprite(i){
    if (i === 0) {
      car.texture = text0;
    } else if (i === 1) {
      car.texture = text1;
    } else if (i === 2) {
      car.texture = text2;
    } else if (i === 3) {
      car.texture = text3;
    } else if (i === 4) {
      car.texture = text4;
    }
  }
  // Checks if x between min and max.
  function between(x, min, max) {
    return x >= min && x <= max;
  }
  // decides which sprite to display
  function new_sprite(yaw, curr_sprite){
    let res = new Object();
    res.value = false;
    res.sprite = curr_sprite;
    d_yaw = (yaw-last_yaw)/dt;
    if (between(d_yaw,-0.05, 0.05)) {
      if (curr_sprite !== 2) {
        res.value = true; res.sprite = 2;
        curr_sprite = 2;
      }
    } else if (between(d_yaw, 0.05, 0.3)) {
      if (curr_sprite !== 1) {
        res.value = true; res.sprite = 1;
        curr_sprite = 1;
      }
    } else if (between(d_yaw, 0.3, 0.6)) {
      if (curr_sprite !== 0) {
        res.value = true; res.sprite = 0;
        curr_sprite = 0;
      }
    } else if (between(d_yaw, -0.3, -0.05)) {
      if (curr_sprite !== 3) {
        res.value = true; res.sprite = 3;
        curr_sprite = 3;
      }
    } else if (between(d_yaw, -0.6, -0.3)) {
      if (curr_sprite !== 4) {
        res.value = true; res.sprite = 4;
        curr_sprite = 4;
      }
    }
    return res;
  }

  // updates position of car.
  function drive(){
    let coords = coordinate_transform(x, y, yaw);
    car.position.set(coords.x, coords.y);
    car.rotation = coords.yaw;

    let response = new_sprite(yaw, curr_sprite);
    if (response.value) {
      change_sprite(response.sprite);
    }
    for (var i = 0; i < obstacles.length; i++) {
      if (collision(car, obstacles[i])) {
        hit = true;
      } else {
        // There's no collision
      }
    }
  }
  // checks if car is at goal.
  function atGoal(){
    let dist = Math.sqrt(Math.pow(x-goal.x,2) + Math.pow(y-goal.y,2));
    if (dist < marg){
      return true;
    } else {
      return false;
    }
  }

  // checks for collision between two pixi sprites.
  function collision(r1, r2) {
    //Define the variables we'll need to calculate
    let hit, combinedHalfWidths, combinedHalfHeights, vx, vy;
    //hit will determine whether there's a collision
    hit = false;
    //Find the center points of each sprite
    r1.centerX = r1.x + r1.width / 2;
    r1.centerY = r1.y + r1.height / 2;
    r2.centerX = r2.x + r2.width / 2;
    r2.centerY = r2.y + r2.height / 2;

    //Find the half-widths and half-heights of each sprite
    r1.halfWidth = r1.width / 2;
    r1.halfHeight = r1.height / 2;
    r2.halfWidth = r2.width / 2;
    r2.halfHeight = r2.height / 2;

    //Calculate the distance vector between the sprites
    vx = r1.centerX - r2.centerX;
    vy = r1.centerY - r2.centerY;

    //Figure out the combined half-widths and half-heights
    combinedHalfWidths = r1.halfWidth + r2.halfWidth;
    combinedHalfHeights = r1.halfHeight + r2.halfHeight;

    //Check for a collision on the x axis
    if (Math.abs(vx) < combinedHalfWidths) {
      //A collision might be occurring. Check for a collision on the y axis
      if (Math.abs(vy) < combinedHalfHeights) {
        //There's definitely a collision happening
        hit = true;
      } else {
        //There's no collision on the y axis
        hit = false;
      }
    } else {
      //There's no collision on the x axis
      hit = false;
    }
    //`hit` will be either `true` or `false`
    return hit;
  };

  /* Called when run code button is pressed.
  Calls server to start simulation.
  Listens for coordinate updates and moves the car in the simulation accordingly. */

  function runSimulation(onCar){
    app.ticker.start()
    // var socket = io();
    curr_sprite = 5;
    hit = false;
    x = start.x;
    y = start.y;
    yaw = start.yaw;
    if (onCar){
      socket.emit('runCodeOnCar', JSON.stringify({'id': unique_id, 'goal': goal}));
      console.log('Running code on car');
    } else {
      console.log('Connected');
      console.log('Session id: ' + unique_id); // Unique id for each html page opened.
      socket.emit('simulationOnly', JSON.stringify({'id': unique_id, 'goal': goal}));
      console.log('starting simulation');
    }
    // recieves position of car. Updates car position in simulation.
    socket.on('position-sent-car', function(msg){
      if(onCar){
        let = JSON.parse(msg);
        x = obj.x;
        y = obj.y;
        last_yaw = yaw;
        yaw = obj.yaw;
      }
    });
    // recieves position. Updates car position in simulation.
    socket.on('position-sent-sim', function(msg){
      if (onCar == false){
        let obj = JSON.parse(msg);
        x = obj.x;
        y = obj.y;
        last_yaw = yaw;
        yaw = obj.yaw;

        if (hit){
          socket.emit('collision');
          console.log('Collision!');
        }
        if (cancel){
          socket.emit('cancel');
          return
        }
      }
    });
    socket.on('close', function(){
      app.ticker.stop();
      return
      console.log('Connection closed');
      // socket.disconnect(true); // close connection to server.
    });
    // This function is run at 60 Hz. Defined by app.ticker.
    app.ticker.add(delta => drive(delta));
  }
