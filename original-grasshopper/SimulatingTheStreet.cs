using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.Linq;
using System.Drawing;

/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  private void RunScript(bool reset, bool On, int numPeds, int numVehs, double neighDist, double crossScanDist, double vehScanDist, List<Point3d> shops, List<Point3d> stationEnt, List<Point3d> stationExit, Point3d busStop, Point3d vehBusStop, List<Point3d> streetEnds, List<Point3d> objects, List<Point3d> signals, Point3d vehStart0, Point3d vehStart1, List<Point3d> vehEnds0, List<Point3d> vehEnds1, ref object A, ref object B, ref object C, ref object D)
  {
    if (!init || reset){
      rnd = new Random();
      mySim = new Sim(
        numPeds,
        numVehs,
        shops,
        stationEnt,
        stationExit,
        busStop,
        streetEnds,
        objects,
        vehBusStop,
        vehStart0,
        vehEnds0,
        vehStart1,
        vehEnds1);
      init = true;
    }
    if (On){
      mySim.Update(neighDist, objects, crossScanDist, signals, vehScanDist);
      groupRects.Clear();
      vehRects.Clear();
      // peds group box drawing
      var uniqueIds = mySim.myPeds.Select(p => p.groupId).Distinct();
      // ped group rects
      foreach (var id in uniqueIds){
        var members = mySim.myPeds.Where(p => p.groupId == id).ToList();

        Point3d centre = new Point3d(0, 0, 0);
        foreach(var m in members){ centre.X += m.pos.X; centre.Y += m.pos.Y; }
        centre.X /= members.Count;
        centre.Y /= members.Count;

        Vector3d xAxis = new Vector3d(members[0].direction, 0, 0);
        Vector3d yAxis = new Vector3d(0, 1, 0);
        Plane rectPlane = new Plane(centre, xAxis, yAxis);

        double minX = members.Min(m => m.pos.X) - 0.3 - centre.X;
        double maxX = members.Max(m => m.pos.X) + 0.3 - centre.X;
        double minY = members.Min(m => m.pos.Y) - 0.3 - centre.Y;
        double maxY = members.Max(m => m.pos.Y) + 0.3 - centre.Y;

        groupRects.Add(new Rectangle3d(rectPlane, new Interval(minX, maxX), new Interval(minY, maxY)));
      }

      // veh rects
      foreach(var veh in mySim.myVehs){
        Vector3d dir = veh.vel;
        if (dir.Length < 0.001) dir = new Vector3d(veh.direction, 0, 0);
        dir.Unitize();
        Vector3d perp = Vector3d.CrossProduct(Vector3d.ZAxis, dir);
        Plane rectPlane = new Plane(veh.pos, dir, perp);

        double len = veh.isBus ? 10.0 : 5.0;
        double wid = veh.isBus ? 1.25 : 1.0;
        vehRects.Add(new Rectangle3d(rectPlane, new Interval(-len, 0), new Interval(-wid, wid)));
      }
    }

    A = mySim.myPeds.Where(x => !x.isReckless).Select(x => x.pos).ToList();
    B = mySim.myPeds.Where(x => !x.isReckless).Select(x => x.vel).ToList();
    C = mySim.myVehs.Select(x => x.pos).ToList();
    D = mySim.myVehs.Select(x => x.vel).ToList();
    E = groupRects;
    F = vehRects;
    G = mySim.myPeds.Where(x => x.isReckless).Select(x => x.pos).ToList();
    H = mySim.myPeds.Where(x => x.isReckless).Select(x => x.vel).ToList();
  }

  /**/
  bool init = false;
  public static Random rnd;
  List<Rectangle3d> groupRects = new List<Rectangle3d>();
  List<Rectangle3d> vehRects = new List<Rectangle3d>();
  Sim mySim;

  public class Pedestrians{

    // pos,vel, acc, and speed
    public Point3d pos;
    public Vector3d vel;
    public Vector3d acc;
    public double pedSpeed;

    // preset values
    public double maxPedSpeed = 1.5;
    public double minPedSpeed = 1.1;
    // public double crossScanDist = 20.0; // just using input
    // 2.2 s (from Zhao et al, 2019)* 8.94 m/s (speed limit 20 mph)~= 19.7,
    // then if there is no car in range of 20m circle, peds start to cross.
    // they mentioned that pedestrians accept a gap with 95% confidFence when gap > 2.2s (1-lane, ~4m crossing)

    //destination
    public List<Point3d> destinations; // set when spawn
    public List<string> destinationTypes; // "crossing", "bus", "final"
    public int destinationIndex = 0;
    public int direction; // set simultaneously

    //state
    public string state = "moving"; // moving or waiting or crossing or disappear.
    public string spawnSide; // south or north
    public bool isReckless; // 5% chance set at spawn
    public bool isDead = false; // appear or disappear
    public string stopReason = "";
    // if it stop (at a crossingpoint or bus stop), it will be assigned
    public int laneIndex;
    // 0-7, which walking lane (6m of width is devided by 7)

    //group
    public int groupSize; // 1,2,3,4
    public int groupId; // shared ID with group members who share speed, destination

    public Pedestrians(
      Point3d spawnPos,
      List<Point3d> destinations,
      List<string> destinationTypes,
      double pedSpeed,
      int laneIndex,
      int groupSize,
      int groupId,
      bool isReckless,
      string spawnSide)
    {
      this.pos = spawnPos;
      this.pedSpeed = pedSpeed;
      if (destinations[destinations.Count - 1].X > spawnPos.X) this.direction = 1;
      else this.direction = -1;
      this.vel = new Vector3d(direction * this.pedSpeed, 0, 0);

      this.destinations = destinations;
      this.destinationTypes = destinationTypes;
      this.laneIndex = laneIndex;
      this.spawnSide = spawnSide;
      this.isReckless = isReckless;

      this.groupSize = groupSize;
      this.groupId = groupId;
    }

    public void moving (List<Pedestrians> allPeds, double neighDist, List<Point3d> objects){
      this.SFM(allPeds, neighDist, objects);
      // state control
      if (destinationIndex < destinations.Count){
        Point3d current = destinations[destinationIndex];
        if (pos.DistanceTo(current) < 1.5){ // close enough
          string type = destinationTypes[destinationIndex];
          if (type == "crossing"){
            state = "waiting";
            stopReason = "crossing";
          }else if (type == "bus"){
            state = "waiting";
            stopReason = "bus";
          }else if (type == "final"){
            state = "disappear";
          }
          destinationIndex++;
        }
      }
    }

    public void waiting (List<Vehicles> allVehs, double crossScanDist){
      // stay until no cars in the range
      this.vel = Vector3d.Zero;
      bool isVeh = false;
      bool isBus = false;
      double min = double.MaxValue;

      Vehicles closestVeh = null;
      foreach(var veh in allVehs){
        var vehDist = this.pos.DistanceTo(veh.pos);
        if (vehDist < min){
          min = vehDist;
          closestVeh = veh;
        }
      }

      // cross if there is no vehicles.
      if (stopReason == "crossing"){
        if (!isReckless){
          if (min < crossScanDist) isVeh = true;
          if (!isVeh){
            this.state = "crossing";
          }else{
            if (closestVeh != null && closestVeh.state == "stopping") this.state = "crossing";
            else this.state = "waiting";
          }
        }else {
          if (min < 0.7 * crossScanDist) isVeh = true;
          if (!isVeh){
            this.state = "crossing";
          }else{
            if (closestVeh != null && closestVeh.state == "stopping") this.state = "crossing";
            else this.state = "waiting";
          }
        }
      }

      // wait until a bus comes.
      if (stopReason == "bus"){
        foreach(var veh in allVehs){
          if (veh.isBus && veh.state == "stopping"){
            isBus = true;
            break;
          }
        }
        // not done yet.
        if (isBus) this.state = "disappear";
        else this.state = "waiting";
      }
    }

    // go to the other side.
    public void crossing (){
      if (this.spawnSide == "south"){
        if (pos.Y > 29.0){
          spawnSide = "north";
          state = "moving";
        }
      }
      if (this.spawnSide == "north"){
        if (pos.Y < 20.0){
          spawnSide = "south";
          state = "moving";
        }
      }
    }

    // get inside the shop, go through the street, or ride on a bus.
    public void disappear (){
      this.isDead = true;
    }

    // Social Force Model (SFM)
    // Helbing, D. and Molnár, P. (1995) 'Social force model for pedestrian dynamics', Physical Review E, 51(5), pp. 4282–4286.
    public void SFM(List<Pedestrians> allPeds, double neighDist, List<Point3d> objects){
      var steer = Vector3d.Zero;
      var ahead = destinations[destinationIndex] - this.pos;
      var Dist = Math.Abs(this.pos.DistanceTo(destinations[destinationIndex]));
      ahead /= Dist;
      steer += ahead;
      // other pedestrians
      foreach(var ped in allPeds){
        if (ped == this || ped.groupId == this.groupId) continue;
        var d = this.pos.DistanceTo(ped.pos);
        var away = this.pos - ped.pos;
        away /= Math.Pow(d, 1.5);

        if(d > 0 && d < neighDist){
          steer += away;
        }
      }
      // objects
      foreach (var obj in objects){
        var away = this.pos - obj;
        var objDist = this.pos.DistanceTo(obj);
        away /= (objDist * objDist);
        if (objDist > 0 && objDist < 1.5){
          steer += away;
        }
      }
      // clamp
      double y;
      if (this.spawnSide == "south"){
        if (this.pos.Y > 17) y = 20;
        else y = 14;
      }else{
        if (this.pos.Y > 32) y = 35;
        else y = 29;
      }
      Point3d edgeP = new Point3d(this.pos.X, y, 0);
      var edgeDist = this.pos.DistanceTo(edgeP);
      var edge = this.pos - edgeP;
      edge /= edgeDist;
      if (edgeDist > 0 && edgeDist < 1.5){
        steer += edge;
      }
      if(steer.Length > 0){
        steer.Unitize();
        steer *= this.pedSpeed;
        steer -= this.vel;
      }
      this.acc += steer;
    }

    public void Update(
      List<Pedestrians> allPeds,
      double neighDist,
      List<Vehicles> allVehs,
      List<Point3d> objects,
    double crossScanDist){

      // moving
      if (state == "moving") {
        // update state
        moving(allPeds, neighDist, objects);
        // update pos, vel, acc
        this.vel += this.acc;
        if(this.vel.Length > this.pedSpeed){
          this.vel.Unitize();
          this.vel *= this.pedSpeed;
        }
        this.pos += this.vel;
        if (spawnSide == "south"){ // clamp
          this.pos.Y = Math.Max(14.0, Math.Min(this.pos.Y, 20.0));
        }else{
          this.pos.Y = Math.Max(29.0, Math.Min(this.pos.Y, 35.0));
        }
        this.acc = Vector3d.Zero;
      }

        // waiting
      else if (state == "waiting") {
        // update state
        waiting(allVehs, crossScanDist);
        // update pos, vel, acc
        this.vel = Vector3d.Zero;
        this.acc = Vector3d.Zero;
      }

        // crossing
      else if (state == "crossing") {
        // update state
        crossing();
        // update pos, vel, acc
        if (this.spawnSide == "south"){
          this.vel = new Vector3d(0, 1.5, 0);
          this.pos += this.vel;
          this.acc = Vector3d.Zero;
        }else if(this.spawnSide == "north"){
          this.vel = new Vector3d(0, -1.5, 0);
          this.pos += this.vel;
          this.acc = Vector3d.Zero;
        }
      }

        // disappear
      else if (state == "disappear") disappear();
    }
  }



  public class Vehicles{

    // pos,vel, acc, and speed
    public Point3d pos;
    public Vector3d vel;
    public double vehSpeed;
    public double spawnY;

    // preset values
    public double maxVehSpeed = 9.0; // max ~= 8.94 m/s (speed limit 20 mph)
    public double minVehSpeed = 8.0;
    public double frontScanDist = 20.0; // using input?

    // destination
    public List<Point3d> destinations; // set when spawn
    public int direction;
    public int destinationIndex = 0;
    public List<string> destinationTypes; // "snap", "bus", or "final"

    // state
    public string state = "moving"; // moving, stopping, or disappear.
    public string waitReason = ""; // "bus", "turning"
    public bool isDead = false; // appear or disappear
    public bool isBus = false; // bus or not.
    public int count = 0;
    public int timer = 0;

    public List<Vehicles> nearVeh = new List<Vehicles>();

    public Vehicles(
      Point3d spawnPos,
      List<Point3d> destinations,
      List<string> destinationTypes,
      double vehSpeed,
      bool isBus
    ){
      this.pos = spawnPos;
      this.spawnY = spawnPos.Y;
      this.vehSpeed = vehSpeed;
      if (destinations[destinations.Count - 1].X > spawnPos.X){
        direction = 1;
      } else {
        direction = -1;
      }
      this.vel = new Vector3d(direction * this.vehSpeed, 0, 0);
      this.destinations = destinations;
      this.destinationTypes = destinationTypes;
      this.isBus = isBus;
    }

    public bool scanning(
      List<Pedestrians> allPeds,
      List<Vehicles> allVehs,
      List<Point3d> signals,
    double vehScanDist){
      nearVeh.Clear();
      int num = 0;

      // scan pedestrians in rectangle-like shape.
      foreach (var ped in allPeds){
        bool near = false;
        if (direction == 1 ) {
          if (
            (ped.pos.X > this.pos.X && ped.pos.X < this.pos.X + vehScanDist)
            &&
            (ped.pos.Y > this.pos.Y - 3 && ped.pos.Y < this.pos.Y + 3))
            near = true;
        }else {
          if (
            (ped.pos.X > this.pos.X - vehScanDist && ped.pos.X < this.pos.X)
            &&
            (ped.pos.Y > this.pos.Y - 3 && ped.pos.Y < this.pos.Y + 3))
            near = true;
        }
        if (near && ped.state == "waiting" && ped.stopReason == "crossing")
          num++;
      }

      // scan vehicles in front of you.
      foreach (var veh in allVehs){
        if (this == veh) continue;
        bool near = false;
        if (direction == 1 ) {
          if (
            (veh.pos.X > this.pos.X && veh.pos.X < this.pos.X + vehScanDist)
            &&
            Math.Abs(this.pos.Y - veh.pos.Y) < 2)
            near = true;
        }else {
          if (
            (veh.pos.X > this.pos.X - vehScanDist && veh.pos.X < this.pos.X)
            &&
            Math.Abs(this.pos.Y - veh.pos.Y) < 2)
            near = true;
        }
        double angleV = Vector3d.VectorAngle(this.vel, new Vector3d(veh.pos - this.pos));
        if (near && angleV < 90){
          nearVeh.Add(veh);
          num++;
        }
      }

      // check signals
      if (direction == 1){
        if (count % 40 > 20 && this.pos.DistanceTo(signals[0]) < 9){
          num++;
        }
      }else{
        if (count % 40 > 20 && this.pos.DistanceTo(signals[1]) < 9){
          num++;
        }
      }
      if (num > 0) return false;
      else return true;
    }

    public void moving (bool isClear){
      // state control
      if (!isClear) state = "stopping";
      else if (destinationIndex < destinations.Count){
        Point3d current = destinations[destinationIndex];
        if (pos.DistanceTo(current) < 9){ // close enough
          string type = destinationTypes[destinationIndex];
          if (type == "snap"){
            state = "waiting";
          }else if (type == "bus"){
            state = "stopping";
          }else if (type == "final"){
            state = "disappear";
          }
          destinationIndex++;
        }
      }
    }

    public void waiting(bool isClear){
      if(isClear){
        state = "moving";
      }
    }


    public void stopping(List<Point3d> signals, bool isClear){
      bool nearSignal = false;
      if (destinationIndex < destinationTypes.Count){
        string type = destinationTypes[destinationIndex];
        if (direction == 1){
          if (pos.DistanceTo(signals[0]) < 9) nearSignal = true;
        }else{
          if (pos.DistanceTo(signals[1]) < 9) nearSignal = true;
        }

        // for cars
        if (!isBus){
          // scan peds and vehs
          if (isClear){
            // at the signal
            if (nearSignal){
              if (count % 40 < 20){
                state = "moving";
              }
            }else state = "moving";
          }
          // for buses
        }else if (type == "bus"){
          timer++; //
          if (timer > 30 && isClear) {
            state = "moving";
            timer = 0;
          }
        }else if(isClear) state = "moving";
      }
    }

    // get to the end.
    public void disappear (){
      this.isDead = true;
    }

    public void Update(
      List<Pedestrians> allPeds,
      List<Vehicles> allVehs,
      List<Point3d> signals,
    double vehScanDist){

      bool isClear = scanning(allPeds, allVehs, signals, vehScanDist);

      // moving
      if (state == "moving"){
        moving(isClear);
        // keep same speed.
        if (destinationIndex < destinations.Count){
          if (nearVeh.Count > 0 && nearVeh[0].vel.Length < this.vel.Length){
            this.vel = nearVeh[0].vel;
          }else{
            this.vel = destinations[destinationIndex] - this.pos;
            this.vel.Unitize();
            this.vel *= vehSpeed;
          }
          this.pos += this.vel;
        }

        // waiting
      }else if (state == "waiting"){
        Point3d snap = destinations[destinationIndex - 1];
        this.pos = new Point3d(snap.X, this.spawnY, 0);
        this.vel = Vector3d.Zero;
        waiting(isClear);

        // stopping
      }else if (state == "stopping"){
        stopping(signals, isClear);
        this.vel = Vector3d.Zero;

        // disappear
      }else if (state == "disappear") disappear();

      // count up.
      count++;
    }
  }




  public class Sim{
    public List<Pedestrians> myPeds;
    public List<Vehicles> myVehs;
    public List<Vehicles> myPendingVehs;
    public double maxPedSpeed = 1.5;
    public double minPedSpeed = 1.1;
    public double maxVehSpeed = 9.0;
    public double minVehSpeed = 8.0;

    public Sim(
      int numPeds, int numVehs,
      List<Point3d> shops,
      List<Point3d> stationEnt,
      List<Point3d> stationExit,
      Point3d busStop,
      List<Point3d> streetEnds,
      List<Point3d> objects,
      Point3d vehBusStop,
      Point3d vehStart0,
      List<Point3d> vehEnds0,
      Point3d vehStart1,
      List<Point3d> vehEnds1
    ){
      // lists for peds
      List<Point3d> allPedsSpawns = new List<Point3d>();
      List<Point3d> allPedsDests = new List<Point3d>();
      List<string> allPedsDestTypes = new List<string>();

      // shops
      foreach (var p in shops){
        allPedsSpawns.Add(p);
        allPedsDests.Add(p);
        allPedsDestTypes.Add("final");
      }
      // station entrance
      foreach (var p in stationEnt){
        allPedsDests.Add(p);
        allPedsDestTypes.Add("final");
      }
      // station exit
      foreach (var p in stationExit){
        allPedsSpawns.Add(p);
      }
      // bus stop
      allPedsSpawns.Add(busStop);
      allPedsDests.Add(busStop);
      allPedsDestTypes.Add("bus");
      // ends
      foreach (var p in streetEnds){
        allPedsSpawns.Add(p);
        allPedsDests.Add(p);
        allPedsDestTypes.Add("final");
      }
      myPeds = new List<Pedestrians>();

      // lists for vehs
      List<Point3d> allVehDests0 = new List<Point3d>();
      List<Point3d> allVehDests1 = new List<Point3d>();

      foreach (var p in vehEnds0){
        allVehDests0.Add(p);
      }
      foreach (var p in vehEnds1){
        allVehDests1.Add(p);
      }


      // peds generation
      // group making
      int groupIdCounter = 0;
      int totalSpawned = 0;

      while (totalSpawned < numPeds){

        double roll = rnd.NextDouble();
        int groupSize;
        if (roll < 0.20) groupSize = 1;
        else if (roll < 0.60) groupSize = 2;
        else if (roll < 0.90) groupSize = 3;
        else                  groupSize = 4;
        int startLane = rnd.Next(0, 7 - groupSize + 1);

        // final destination
        int destIdx = rnd.Next(allPedsDests.Count);
        Point3d finalDest = allPedsDests[destIdx];
        string finalType = allPedsDestTypes[destIdx];

        // spawn
        Point3d spawn = allPedsSpawns[rnd.Next(allPedsSpawns.Count)];
        string spawnSide = (spawn.Y < 25) ? "south" : "north";
        double speed = minPedSpeed + rnd.NextDouble() * (maxPedSpeed - minPedSpeed);

        for (int m = 0; m < groupSize; m++){
          // spawn.Y
          int laneIndex = startLane + m;
          double spawnY;
          if (spawnSide == "south"){
            spawnY = 14.6 + 0.8 * laneIndex;
          }else{
            spawnY = 29.6 + 0.8 * laneIndex;
          }
          Point3d spawnPos = new Point3d(spawn.X, spawnY, 0);

          //waypoint list
          var dests = new List<Point3d>();
          var types = new List<string>();
          bool needsCrossing = Math.Abs(finalDest.Y - spawnPos.Y) > 9.0;
          if (needsCrossing){
            double crossingY = (spawnSide == "south") ? 20.0 : 29.0;
            double crossingX = finalDest.X - 5 + rnd.NextDouble() * 10;
            dests.Add(new Point3d(crossingX, crossingY, 0));
            types.Add("crossing");
          }
          dests.Add(finalDest);
          types.Add(finalType);

          // reckless
          bool isReckless = rnd.NextDouble() < 0.05;

          // build
          myPeds.Add(new Pedestrians(
            spawnPos, dests, types,
            speed,
            laneIndex, // laneIndex
            groupSize, // groupSize
            groupIdCounter,
            isReckless,
            spawnSide));

          totalSpawned++;
          if (totalSpawned >= numPeds) break;
        }
        groupIdCounter++;
      }


      // vehs generation
      myPendingVehs = new List<Vehicles>();
      myVehs = new List<Vehicles>();

      //from right to left: 0
      for (int i = 0;i < numVehs / 2; i++){
        // final destination
        int destIdx = rnd.Next(allVehDests0.Count);
        Point3d finalDest = allVehDests0[destIdx];
        string finalType = "final";

        // spawn
        Point3d spawnPos = vehStart0;
        double speed = minVehSpeed + rnd.NextDouble() * (maxVehSpeed - minVehSpeed);

        // destinations
        var dests = new List<Point3d>();
        var types = new List<string>();

        // is Bus
        bool isBus = rnd.NextDouble() < 0.05;
        if (isBus){
          dests.Add(vehBusStop);
          types.Add("bus");
        }

        // snap
        bool needsTurning = Math.Abs(finalDest.Y - spawnPos.Y) > 0.0;
        if (needsTurning){
          dests.Add(new Point3d(finalDest.X, spawnPos.Y, 0));
          types.Add("snap");
        }
        dests.Add(finalDest);
        types.Add(finalType);


        // build
        myPendingVehs.Add(new Vehicles(
          spawnPos,
          dests,
          types,
          speed,
          isBus
          ));
      }

      // from left to right: 1
      for (int i = 0; i < numVehs / 2;i++){
        // final destination
        int destIdx = rnd.Next(allVehDests1.Count);
        Point3d finalDest = allVehDests1[destIdx];
        string finalType = "final";

        // spawn
        Point3d spawnPos = vehStart1;
        double speed = minVehSpeed + rnd.NextDouble() * (maxVehSpeed - minVehSpeed);

        // destinations
        var dests = new List<Point3d>();
        var types = new List<string>();

        // is Bus
        bool isBus = rnd.NextDouble() < 0.05;

        // snap
        bool needsTurning = Math.Abs(finalDest.Y - spawnPos.Y) > 0.0;
        if (needsTurning){
          dests.Add(new Point3d(finalDest.X, spawnPos.Y, 0));
          types.Add("snap");
        }
        dests.Add(finalDest);
        types.Add(finalType);


        // build
        myPendingVehs.Add(new Vehicles(
          spawnPos,
          dests,
          types,
          speed,
          isBus
          ));
      }
      // shuffle the order of vehs in this list
      for (int i = myPendingVehs.Count - 1; i > 0; i--){
        int j = rnd.Next(i + 1);
        var temp = myPendingVehs[i];
        myPendingVehs[i] = myPendingVehs[j];
        myPendingVehs[j] = temp;
      }
    }


    public void Update(double neighDist, List<Point3d> objects, double crossScanDist, List<Point3d> signals, double vehScanDist){
      foreach(var ped in myPeds){
        ped.Update(myPeds, neighDist, myVehs, objects, crossScanDist);
      }
      myPeds.RemoveAll(p => p.isDead);
      for (int i = myPendingVehs.Count - 1; i >= 0; i--){
        if (rnd.NextDouble() < 0.50){
          myVehs.Add(myPendingVehs[i]);
          myPendingVehs.RemoveAt(i);
          break;
        }
      }
      foreach (var veh in myVehs){
        veh.Update(myPeds, myVehs, signals, vehScanDist);
      }
      myVehs.RemoveAll(p => p.isDead);
    }
  }
  /**/
} 