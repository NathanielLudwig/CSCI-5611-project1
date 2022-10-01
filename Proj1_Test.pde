//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 0;
int numNodes  = 100;

int total = 0;
int times = 0;
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);

Vec2 agentPos;
Vec2 agentVel;
float agentRadius = 20;
float agentRotation = 0;
float agentTargetRotation = 0;
float speed = 100;

PImage cloud;
PImage plane;

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3) + agentRadius);
  }
  circleRad[0] = 30; //Make the first obstacle big
}

ArrayList<Integer> curPath;

int strokeWidth = 2;
void setup(){
  size(1024,768, P2D);
  cloud = loadImage("assets/cloud.png");
  plane = loadImage("assets/plane.png");
  testPRM();
}

int numCollisions;
float pathLength;
boolean reachedGoal;
void pathQuality(){
  Vec2 dir;
  hitInfo hit;
  float segmentLength;
  numCollisions = 9999; pathLength = 9999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return; //No path found  
  
  pathLength = 0; numCollisions = 0;
  
  if (curPath.size() == 0 ){ //Path found with no nodes (direct start-to-goal path)
    segmentLength = startPos.distanceTo(goalPos);
    pathLength += segmentLength;
    dir = goalPos.minus(startPos).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
    if (hit.hit) numCollisions += 1;
    return;
  }
  
  segmentLength = startPos.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  dir = nodePos[curPath.get(0)].minus(startPos).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
  if (hit.hit) numCollisions += 1;
  
  
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;
    
    dir = nodePos[nextNode].minus(nodePos[curNode]).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[curNode], dir, segmentLength);
    if (hit.hit) numCollisions += 1;
  }
  
  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(goalPos);
  pathLength += segmentLength;
  dir = goalPos.minus(nodePos[lastNode]).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[lastNode], dir, segmentLength);
  if (hit.hit) numCollisions += 1;
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  }
  return randPos;
}

void testPRM(){
  long startTime, endTime;
  
  //placeRandomObstacles(numObstacles);
  
  startPos = sampleFreePos();
  goalPos = sampleFreePos();
  agentPos = new Vec2(startPos.x, startPos.y);

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  startTime = System.nanoTime();
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  endTime = System.nanoTime();
  pathQuality();
  total += pathLength;
  times += 1;
  println("Avg", total / times);
  println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
          " Path Len:", pathLength, " Path Segment:", curPath.size()+1,  " Num Collisions:", numCollisions);
}

boolean paused = true;
void draw(){
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(111, 170, 230); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  //Update agent if not paused
  if (!paused && curPath.size() > 0){
    moveAgent(1.0/frameRate);
  }
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i] - agentRadius;
    pushMatrix();
    textureMode(NORMAL);
    noStroke();
    translate(c.x, c.y);
    beginShape();
    texture(cloud);
    vertex(-r, -r, 0, 0);
    vertex(r, -r, 1,   0); //<>//
    vertex(r, r, 1, 1);
    vertex(-r, r, 0, 1);
    endShape();
    popMatrix();
    noFill();
    circle(c.x,c.y,r*2);
    stroke(0, 0, 0);
  }
  ////Draw the first circle a little special b/c the user controls it
  //fill(240);
  //strokeWeight(2);
  //circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  //strokeWeight(1);
  
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
    circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      stroke(0, 5);
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }

  //Draw Start and Goal
  fill(20,60,250);
  //circle(nodePos[startNode].x,nodePos[startNode].y,20);
  //circle(startPos.x,startPos.y,20);
  fill(250,30,50);
  //circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
  circle(goalPos.x,goalPos.y,20);
  
  if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found
  
  //Draw Planned Path
  stroke(255,255,255,150);
  strokeWeight(3);
  if (curPath.size() == 0){
    line(startPos.x,startPos.y,goalPos.x,goalPos.y);
    return;
  }
  line(startPos.x,startPos.y,nodePos[curPath.get(0)].x,nodePos[curPath.get(0)].y);
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
  }
  line(goalPos.x,goalPos.y,nodePos[curPath.get(curPath.size()-1)].x,nodePos[curPath.get(curPath.size()-1)].y);
    // draw agent
  pushMatrix();
  textureMode(NORMAL);
  noStroke();
  translate(agentPos.x, agentPos.y);
  rotate(agentRotation);
  beginShape();
  texture(plane);
  vertex(-agentRadius, -agentRadius, 0, 0);
  vertex(agentRadius, -agentRadius, 1,   0);
  vertex(agentRadius, agentRadius, 1, 1);
  vertex(-agentRadius, agentRadius, 0, 1);
  endShape();
  popMatrix();
  noFill();
  circle(agentPos.x, agentPos.y, agentRadius * 2);
}

boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    testPRM();
    return;
  }
  if (key == 'a'){
    circlePos[numObstacles] = new Vec2(mouseX, mouseY);
    circleRad[numObstacles] = 60 + agentRadius;
    numObstacles++;
    testPRM();
  }
  if (key == ' ') paused = !paused;

  if (keyCode == SHIFT){
    shiftDown = true;
  }
  Vec2 mousepos = new Vec2(mouseX, mouseY);
  int circleidx = pointInCircleListidx(circlePos , circleRad ,numObstacles, mousepos, 2);

  float speed = 10;
  if (shiftDown) speed = 30;
  
  if (keyCode == RIGHT){
    circlePos[circleidx].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[circleidx].x -= speed;
  }
  if (keyCode == UP){
    circlePos[circleidx].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[circleidx].y += speed;
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}

void mousePressed(){
  if (mouseButton == RIGHT){
    startPos = new Vec2(mouseX, mouseY);
    agentPos = new Vec2(mouseX, mouseY);
  }
  else{ //<>//
    goalPos = new Vec2(mouseX, mouseY);
    //println("New Goal is",goalPos.x, goalPos.y);
  }
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

void moveAgent(float dt){
  //Compute accelerations for every agents
  Vec2 agentGoalPos = agentPos;
  if (!checkCollision(agentPos, goalPos)) {
    agentGoalPos = goalPos;
  } else {
    for (int i = curPath.size() - 1; i >= 0; i--){
      if (!checkCollision(agentPos, nodePos[curPath.get(i)])){
        agentGoalPos = nodePos[curPath.get(i)];
        break;
      }
    }
  }
  Vec2 dir = agentGoalPos.minus(agentPos);
  if (abs(dir.x) < 2 && abs(dir.y) < 2) {
    return;
  }
  if (dir.length() > 0) dir.normalize();
  agentVel = dir.times(speed * dt);
  agentPos.add(agentVel);
  if (agentPos.distanceTo(startPos) < 2) {
    agentTargetRotation = atan2(dir.y, dir.x) + PI/2;
    agentRotation = atan2(dir.y, dir.x) + PI/2;
  }

  if (abs(agentTargetRotation - agentRotation) < 0.1) { //<>//
    agentTargetRotation = atan2(dir.y, dir.x) + PI/2;
  } else {
    if (agentTargetRotation - agentRotation < 0) {
      agentRotation +=  -1 * 2 * dt;
    } else {
      agentRotation +=  1 * 2 * dt;
    }
  }
  //agentRotation = atan2(dir.y, dir.x) + PI/2;
}
int pointInCircleListidx(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos, eps)){
      return i;
    }
  }
  return -1;
}
boolean checkCollision(Vec2 node1Pos, Vec2 node2Pos) {
  Vec2 dir = node2Pos.minus(node1Pos).normalized();
    float distBetween = node1Pos.distanceTo(node2Pos);
    hitInfo circleListCheck = rayCircleListIntersect(circlePos, circleRad, numObstacles, node1Pos, dir, distBetween);
    return circleListCheck.hit;
}
