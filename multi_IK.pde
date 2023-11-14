Obstacle[] obstacles;

void setup(){
  size(700,700);
  obstacles = new Obstacle[2];
  obstacles[0] = new Obstacle(new Vec2(160, 200), 40);
  obstacles[1] = new Obstacle(new Vec2(300, 280), 25);
}

//Root
Vec2 root1 = new Vec2(250,160);

//Upper Arm
float l0 = 20; 
float a0 = 0.3; //Shoulder joint

float l0_r = 20; 
float a0_r = 0.3; //Shoulder joint

//Lower Arm
float l1 = 70;
float a1 = 0.3; //Elbow joint

float l1_r = 70; 
float a1_r = 0.3; //Shoulder joint

//Hand
float l2 = 65;
float a2 = 0.3; //Wrist joint

float l2_r = 65;
float a2_r = 0.3; //Wrist joint

//shoulder
float l3 = 20;
float a3 = 0.3; 

float l3_r = 20;
float a3_r = 0.3; 

//finger
float l4 = 14;
float a4 = 0.3; 

float l4_r = 14;
float a4_r = 0.3; 

Vec2 start_l1,start_l2,start_l3,start_l4,endPoint;
Vec2 start_l1_r,start_l2_r,start_l3_r,start_l4_r,endPoint_r;
float old_a4, old_a3, old_a2, old_a1, old_a0;
float old_ar4, old_ar3, old_ar2, old_ar1, old_ar0;

void solve_l(){
  if (controlLeftArm){
    Vec2 goal = new Vec2(mouseX, mouseY);
    Vec2 startToGoal, startToEndEffector;
    float dotProd, angleDiff;

    old_a4 = a4;
    old_a3 = a3;
    old_a2 = a2;
    old_a1 = a1;
    old_a0 = a0;
  
    startToGoal = goal.minus(start_l4);
    startToEndEffector = endPoint.minus(start_l4);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a4 = old_a4;
      if (cross(startToGoal,startToEndEffector) < 0)
        a4 += angleDiff;
      else
        a4 -= angleDiff;
      if(a4 > 1.7){
        a4 = 1.7;
      }
      else if(a4 < -PI/2){
        a4 = -PI/2;
      }
      fk(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
      
    //if(!checkForCollisions(start_l3, start_l4)){
      startToGoal = goal.minus(start_l3);
      startToEndEffector = endPoint.minus(start_l3);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      do {
        a3 = old_a3;
        if (cross(startToGoal,startToEndEffector) < 0)
          a3 += angleDiff;
        else
          a3 -= angleDiff;
        if(a3 > 0.5*PI){
          a3 = 0.5*PI;
        }
        else if(a3 < -0.5*PI){
          a3 = -0.5*PI;
        }
        fk(); //Update link positions with fk (e.g. end effector changed)
        angleDiff *= .5; //Shrink angle difference and try again if there is a collision
      } while (armCircleCollision());
      
    //if(!checkForCollisions(start_l2, start_l3)){
      startToGoal = goal.minus(start_l2);
      startToEndEffector = endPoint.minus(start_l2);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      do{
        a2 = old_a2;
        if (cross(startToGoal,startToEndEffector) < 0)
          a2 += angleDiff;
        else
          a2 -= angleDiff;
        if(a2 > PI/1.2){
          a2 = PI/1.2;
        }
        else if(a2 < -PI/1.2){
          a2 = -PI/1.2;
        }
        fk(); //Update link positions with fk (e.g. end effector changed)
        angleDiff *= .5; //Shrink angle difference and try again if there is a collision
      } while (armCircleCollision());
      
      
      startToGoal = goal.minus(start_l1);
      startToEndEffector = endPoint.minus(start_l1);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      do{
        a1 = old_a1;
        if (cross(startToGoal,startToEndEffector) < 0)
          a1 += angleDiff;
        else
          a1 -= angleDiff;

        fk(); //Update link positions with fk (e.g. end effector changed)
        angleDiff *= .5; //Shrink angle difference and try again if there is a collision
      } while (armCircleCollision());
      
    //if(!checkForCollisions(root1, start_l1)){
      startToGoal = goal.minus(root1);
      if (startToGoal.length() < .0001) return;
      startToEndEffector = endPoint.minus(root1);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      do{
        a0 = old_a0;
        if (cross(startToGoal,startToEndEffector) < 0)
          a0 += angleDiff;
        else
          a0 -= angleDiff;
        fk(); //Update link positions with fk (e.g. end effector changed)
        angleDiff *= .5; //Shrink angle difference and try again if there is a collision
      } while (armCircleCollision());
      
      a0_r = a0;
      fk2();
  }
}

void solve_r(){
  if (!controlLeftArm){
    Vec2 goal = new Vec2(mouseX, mouseY);
    
    Vec2 startToGoal, startToEndEffector;
    float dotProd, angleDiff;
    
    old_ar4 = a4_r;
    old_ar3 = a3_r;
    old_ar2 = a2_r;
    old_ar1 = a1_r;
    old_ar0 = a0_r;
    
    startToGoal = goal.minus(start_l4_r);
    startToEndEffector = endPoint_r.minus(start_l4_r);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a4_r = old_ar4;
      if (cross(startToGoal,startToEndEffector) < 0)
        a4_r += angleDiff;
      else
        a4_r -= angleDiff;
      if(a4_r > 1.7){
        a4_r = 1.7;
      }
      else if(a4_r < -PI/2){
        a4_r = -PI/2;
      }
      fk2(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
    startToGoal = goal.minus(start_l3_r);
    startToEndEffector = endPoint_r.minus(start_l3_r);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a3_r = old_ar3;
      if (cross(startToGoal,startToEndEffector) < 0)
        a3_r += angleDiff;
      else
        a3_r -= angleDiff;
      if(a3_r > 0.5*PI){
        a3_r = 0.5*PI;
      }
      else if(a3_r < -0.5*PI){
        a3_r = -0.5*PI;
      }
      fk2(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
    
    //Update wrist joint
    startToGoal = goal.minus(start_l2_r);
    startToEndEffector = endPoint_r.minus(start_l2_r);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a2_r = old_ar2;
      if (cross(startToGoal,startToEndEffector) < 0)
        a2_r += angleDiff;
      else
        a2_r -= angleDiff;
      /*TODO: Wrist joint limits here*/
      if(a2_r > PI/1.2){
        a2_r = PI/1.2;
      }
      else if(a2_r < -PI/1.2){
        a2_r = -PI/1.2;
      }
      fk2(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
   
    //Update elbow joint
    startToGoal = goal.minus(start_l1_r);
    startToEndEffector = endPoint_r.minus(start_l1_r);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a1_r = old_ar1;
      if (cross(startToGoal,startToEndEffector) < 0)
        a1_r += angleDiff;
      else
        a1_r -= angleDiff;
      fk2(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
    
    //Update shoulder joint
    startToGoal = goal.minus(root1);
    if (startToGoal.length() < .0001) return;
    startToEndEffector = endPoint_r.minus(root1);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = acos(dotProd);
    do {
      a0_r = old_ar0;
      if (cross(startToGoal,startToEndEffector) < 0)
        a0_r += angleDiff;
      else
        a0_r -= angleDiff;
      fk2(); //Update link positions with fk (e.g. end effector changed)
      angleDiff *= .5; //Shrink angle difference and try again if there is a collision
    } while (armCircleCollision());
    
    a0 = a0_r;
    fk();
  
  }
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root1);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  start_l4 = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
  endPoint = new Vec2(cos(a0+a1+a2+a3+a4)*l4,sin(a0+a1+a2+a3+a4)*l4).plus(start_l4);
}

void fk2(){
  start_l1_r = new Vec2(cos(a0_r)*l0_r,sin(a0_r)*l0_r).plus(root1);
  start_l2_r = new Vec2(cos(a0_r+a1_r)*l1_r,sin(a0_r+a1_r)*l1_r).plus(start_l1_r);
  start_l3_r = new Vec2(cos(a0_r+a1_r+a2_r)*l2_r,sin(a0_r+a1_r+a2_r)*l2_r).plus(start_l2_r);
  start_l4_r = new Vec2(cos(a0_r+a1_r+a2_r+a3_r)*l3_r,sin(a0_r+a1_r+a2_r+a3_r)*l3_r).plus(start_l3_r);
  endPoint_r = new Vec2(cos(a0_r+a1_r+a2_r+a3_r+a4_r)*l4_r,sin(a0_r+a1_r+a2_r+a3_r+a4_r)*l4_r).plus(start_l4_r);
}



float armW = 10;
boolean checkForCollisions(Vec2 p1, Vec2 p2) {
  for (Obstacle obstacle : obstacles) {
    float effectiveRadius = obstacle.radius + armW / 2;

    float x1 = obstacle.center.x - p1.x;
    float y1 = obstacle.center.y - p1.y;
    float x2 = obstacle.center.x - p2.x;
    float y2 = obstacle.center.y - p2.y;
    
    float start_dis = x1 * x1 + y1 * y1;
    float end_dis = x2 * x2 + y2 * y2;

    // Check if either end of the segment is inside the circle
    if (start_dis < effectiveRadius * effectiveRadius || end_dis < effectiveRadius * effectiveRadius) {
        return true;
    }
    
    Vec2 ldir = new Vec2(p2.x - p1.x, p2.y - p1.y);
    float lens = ldir.x * ldir.x + ldir.y * ldir.y;
    float len = (float)Math.sqrt(lens);
    ldir = ldir.normalized();
    Vec2 toCircle = new Vec2(obstacle.center.x - p1.x, obstacle.center.y - p1.y);

    float a = 1.0f;
    float b = -2 * dot(ldir, toCircle);
    float lensqr = toCircle.x * toCircle.x + toCircle.y * toCircle.y;
    float c = lensqr - effectiveRadius * effectiveRadius;

    float d = b * b - 4 * a * c;
    if (d >= 0) {
        float sqrtD = (float)Math.sqrt(d);
        float t1 = (-b - sqrtD) / (2 * a);
        float t2 = (-b + sqrtD) / (2 * a);
        // Check if either t1 or t2 lies within the segment
        if ((t1 > 0 && t1 < len) || (t2 > 0 && t2 < len)) {
            return true;
        }
    }
  }

    return false;
}


boolean armCircleCollision() {
    // Check each segment of the arm for collision
    if(controlLeftArm){
      if (checkForCollisions(root1, start_l1)) return true;
      if (checkForCollisions(start_l1, start_l2)) return true;
      if (checkForCollisions(start_l2, start_l3)) return true;
      if (checkForCollisions(start_l3, start_l4)) return true;
      if (checkForCollisions(start_l4, endPoint)) return true;
    }
    if(!controlLeftArm){
      if (checkForCollisions(root1, start_l1_r)) return true;
      if (checkForCollisions(start_l1_r, start_l2_r)) return true;
      if (checkForCollisions(start_l2_r, start_l3_r)) return true;
      if (checkForCollisions(start_l3_r, start_l4_r)) return true;
      if (checkForCollisions(start_l4_r, endPoint_r)) return true;
    }

    return false;
}


boolean run = true;
boolean controlLeftArm = true;
void keyPressed() {
  if (key == 'L' || key == 'l') {
    controlLeftArm = true;
  } 
  else if (key == 'R' || key == 'r') {
    controlLeftArm = false;
  }
  else if (key == ' '){
    run = !run;
  }
}

void draw(){
  fk();
  fk2();
  if (run) {
    if (controlLeftArm) {
      solve_l(); 
    } 
    else {
      solve_r(); 
    }
  }
  
  background(250,250,250);
  
  fill(147,112,219);
  ellipse(250,160, 70, 70);
  
  fill(230,230,250);
  pushMatrix();
  translate(root1.x,root1.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  //left
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/2, l3, armW);
  popMatrix();
    
  pushMatrix();
  translate(start_l4.x,start_l4.y);
  rotate(a0+a1+a2+a3+a4);
  rect(0, -armW/2, l4, armW);
  popMatrix();
  
  // right arm
  pushMatrix();
  translate(start_l1_r.x,start_l1_r.y);
  rotate(a0_r+a1_r);
  rect(0, -armW/2, l1_r, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2_r.x,start_l2_r.y);
  rotate(a0_r+a1_r+a2_r);
  rect(0, -armW/2, l2_r, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3_r.x,start_l3_r.y);
  rotate(a0_r+a1_r+a2_r+a3_r);
  rect(0, -armW/2, l3_r, armW);
  popMatrix();
    
  pushMatrix();
  translate(start_l4_r.x,start_l4_r.y);
  rotate(a0_r+a1_r+a2_r+a3_r+a4_r);
  rect(0, -armW/2, l4_r, armW);
  popMatrix();
  
  for(Obstacle obstacle : obstacles) {
        obstacle.display();
    }
  
}

class Obstacle {
    Vec2 center;
    float radius;

    Obstacle(Vec2 pos, float rad) {
        center = pos;
        radius = rad;
    }
    void display() {
        fill(255,255,255); 
        ellipse(center.x, center.y, radius * 2, radius * 2);
    }
}


public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float magSq() {
    return x * x + y * y;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}



Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
