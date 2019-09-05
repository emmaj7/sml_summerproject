/*
Hold all functions that makes the simulation window work.
eg. functions for checking collisons, changing sprites etc.

Written by: Mikael Glamheden
Last edited: 2019-07-18
*/



// decides which sprite to display
function newSprite(yaw, curr_sprite){
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

// switch which sprite to display.
function changeSprite(i){
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

// Returns the car to the start position
function returnToStart(){
  let coords = coordinateTransform(start_coords.x, start_coords.y, start_coords.yaw);
  car.position.set(coords.x, coords.y);
  car.rotation = coords.yaw;
  hitbox.position.set(coords.x, coords.y);
  hitbox.rotation = coords.yaw;
  console.log('return to start');
  app.ticker.stop();
}
// transforms between "true" coordinate system and browser window coordinates
function coordinateTransform(x, y, yaw){
  let coords = {};
  coords.x = scale_factor*(x + max);
  coords.y = app.renderer.view.height - scale_factor*(y + max); // y = 0 is upper corner
  coords.yaw = -yaw
  return coords;
}
// transform between window coordinates and "real" coordinates
function transformReversed(x, y, yaw){
  let coords = {};
  coords.x = x/scale_factor - max;
  coords.y = -(y - app.renderer.view.height)/scale_factor - max;
  coords.yaw = -yaw;
  return coords;
};

// Checks if float x between min and max.
function between(x, min, max) {
  return x >= min && x <= max;
}

// Checks for collision with borders of map.
function edgeCollision(sprite){
  // Get limits of map
  c1 = transformReversed(0, app.renderer.view.height, 0); // lower left corner
  c2 = transformReversed(app.renderer.view.width, 0, 0); // upper right corner
  corners = getCorners(transformReversed(sprite.x, sprite.y, sprite.rotation),
                       sprite.width/scale_factor,
                       sprite.height/scale_factor);

  // Check for collisions with map border.
  for (var i = 0; i < corners.length; i++) {
    p = corners[i]
    if (p[0] < c1.x || p[0] > c2.x){
      return true
    } else if (p[1] < c1.y || p[1] > c2.y) {
      return true
    }
  }
  return false
}

// returns vector perpendicular to side i of polygon.
function getAxesNormal(corners){
  let axes = [], p1, p2, edge, normal;
  for (var i = 0; i < corners.length; i++) {
    p1 = corners[i];
    p2 = corners[i + 1 == corners.length ? 0 : i + 1];
    edge = subtract(p1, p2);
    normal = getNormal(edge);
    axes[i] = normal;
  }
  return axes;
}
// subtract two vectors of length 2
function subtract(p1, p2){
  let res = [];
  for (var i = 0; i < p1.length; i++) {
    res[i] = p1[i]- p2[i];
  }
  return res;
}
// get normal of a vector of length 2
function getNormal(v){
  let length = 0;
  let sumOfSquares = 0;

  for (var i = 0; i < v.length; i++) {
    sumOfSquares = sumOfSquares + Math.pow(v[i], 2);
  }
  length = Math.sqrt(sumOfSquares);
  return [-v[1]/length, v[0]/length];
}

// get the corners of a rectangle given center point, yaw, width and height.
function getCorners(c, w, h){
  let w_cos = (w/2)*Math.cos(c.yaw);
  let w_sin = (w/2)*Math.sin(c.yaw);
  let h_cos = (h/2)*Math.cos(c.yaw);
  let h_sin = (h/2)*Math.sin(c.yaw);
  let p1 = [c.x + w_cos - h_sin,
            c.y + w_sin + h_cos];
  let p2 = [c.x + w_cos + h_sin,
            c.y + w_sin - h_cos];
  let p3 = [c.x - w_cos - h_sin,
            c.y - w_sin + h_cos];
  let p4 = [c.x - w_cos + h_sin,
            c.y - w_sin - h_cos];
  return [p1, p2, p3, p4];
}

function projectOnAxis(corners, axis){
  let min = dotProduct(corners[0], axis);
  let max = min;
  for (var i = 1; i < corners.length; i++) {
    let p = dotProduct(corners[i], axis);
    if (p < min) {
      min = p;
    } else if (p > max) {
      max = p;
    }
  }
  return {min: min, max: max};
}

function dotProduct(p1, p2){
  return p1[0]*p2[0] + p1[1]*p2[1];
}

// checks for collision between two rectangles (sprites)
// Uses SAT - separating axis theorem.
function collision(r1, r2) {
  let realc1 = transformReversed(r1.x, r1.y, r1.rotation);
  let realc2 = transformReversed(r2.x, r2.y, r2.rotation);
  let w1 = r1.width/scale_factor;
  let h1 = r1.height/scale_factor;
  let w2 = r2.width/scale_factor;
  let h2 = r2.height/scale_factor;

  // corner points of r1
  corners_r1 = getCorners(realc1, w1, h1);
  // corner points of r2
  corners_r2 = getCorners(realc2, w2, h2);

  // get all axis normal to the sides of r1
  axes = getAxesNormal(corners_r1);

  // project the points of r1 and r2 onto each axis.
  for (var i = 0; i < axes.length; i++) {
    let axis = axes[i];
    let proj1 = projectOnAxis(corners_r1, axis);
    let proj2 = projectOnAxis(corners_r2, axis);

    // Check for intersections
    d1 = proj1.min - proj2.max;
    d2 = proj2.min - proj1.max;
    if (d1 > 0 || d2 > 0){
      return false;
    }
  }
  return true;
}
