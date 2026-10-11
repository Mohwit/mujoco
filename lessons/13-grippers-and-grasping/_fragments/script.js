// ======== LESSON 13: GRIPPERS AND GRASPING (sandbox + explainer) ========
(() => {
  // Hold-or-slip grid recorded from MuJoCo 3.15.0 by lessons/13-grippers-and-grasping/record_grip_grid.py.
  // Each run: close 0.5 s, lift 10 cm in 1.0 s (half cosine), hold 0.5 s.
  // sets = squeeze setting per finger (N) = kp (0.030 - ctrl) / 2; meas = measured squeeze per finger (N,
  // touch sensor while a 0.05 kg box is held); masses (kg); mus = finger friction (box is 0.6, MuJoCo uses the max).
  // slip[setIndex][massIndex][muIndex] = box slip relative to the hand at t = 0.1, 0.2, ..., 1.5 s after the
  // lift starts, in units of 0.1 mm. "held" = final slip under 10 mm.
  const G13 = {"sets":[0.25,0.5,0.75,1.0,1.5,2.0,3.0,4.0,6.0,8.0],"masses":[0.1,0.25,0.5,0.75,1.0],"mus":[0.6,0.8,1.0,1.5,2.0],"meas":[0.1962,0.4083,0.6296,0.8535,1.2971,1.7307,2.6004,3.4735,5.2288,6.9918],"slip":[[[[11,67,165,296,447,603,749,870,955,995,999,999,999,999,999],[10,66,165,295,446,602,749,870,955,995,999,999,999,999,999],[10,66,164,295,446,602,749,870,955,995,999,999,999,999,999],[10,65,164,294,444,603,749,870,955,995,999,999,999,999,999],[9,50,139,295,438,601,749,870,955,995,999,999,999,999,999]],[[11,67,165,296,446,602,749,870,955,995,999,999,999,999,999],[11,66,164,295,445,602,749,870,955,995,999,999,999,999,999],[10,66,164,294,445,601,749,870,955,995,999,999,999,999,999],[9,65,163,294,445,602,749,870,955,995,999,999,999,999,999],[8,64,163,293,444,602,749,870,955,995,999,999,999,999,999]],[[11,67,165,296,446,603,749,870,955,995,999,999,999,999,999],[10,66,164,295,446,602,749,870,955,995,999,999,999,999,999],[10,66,164,295,445,602,749,870,955,995,999,999,999,999,999],[9,65,163,294,444,601,749,870,955,995,999,999,999,999,999],[9,64,162,292,443,601,749,870,955,995,999,999,999,999,999]],[[11,67,165,296,446,603,749,870,955,995,999,999,999,999,999],[11,66,165,295,446,602,749,870,955,995,999,999,999,999,999],[10,66,164,295,445,602,749,870,955,995,999,999,999,999,999],[9,65,163,294,444,601,749,870,955,995,999,999,999,999,999],[9,64,162,292,443,600,749,870,955,995,999,999,999,999,999]],[[11,67,165,296,446,603,749,870,955,995,999,1000,1000,1000,1000],[11,66,165,295,446,602,749,870,955,995,999,999,999,999,999],[10,66,164,295,446,602,749,870,955,995,999,999,999,999,999],[9,65,163,294,445,602,749,870,955,995,999,999,999,999,999],[9,64,162,293,444,600,749,870,955,995,999,999,999,999,999]]],[[[9,65,164,294,445,602,748,869,954,995,999,999,999,999,999],[9,64,163,293,444,602,748,869,954,994,999,999,999,999,999],[8,63,162,290,443,602,748,869,954,994,999,999,999,999,999],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,2,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,2,2,2,2,2]],[[9,65,163,294,444,601,748,869,954,995,999,999,999,999,999],[8,64,162,293,444,601,748,869,954,994,999,999,999,999,999],[8,63,162,292,443,600,748,869,954,994,998,999,999,999,999],[6,64,160,291,442,600,748,869,954,994,998,998,998,998,998],[6,61,159,289,440,598,748,869,954,994,998,998,998,998,998]],[[9,65,163,294,445,601,748,869,954,994,999,999,999,999,999],[9,64,162,293,444,600,748,869,954,994,999,999,999,999,999],[8,63,161,292,442,600,748,869,954,994,998,998,998,998,998],[6,61,159,289,441,598,748,869,954,994,998,998,998,998,998],[5,60,158,289,439,598,747,869,954,994,998,998,998,998,998]],[[9,65,163,294,445,601,748,869,954,995,999,999,999,999,999],[9,64,162,293,444,601,748,869,954,994,999,999,999,999,999],[8,63,162,292,443,600,748,869,954,994,998,998,998,998,998],[6,61,160,290,440,598,747,869,954,994,998,998,998,998,998],[5,60,157,287,439,598,747,869,954,994,998,998,998,998,998]],[[9,65,163,294,445,601,748,869,954,995,999,999,999,999,999],[9,64,162,293,444,601,748,869,954,994,999,999,999,999,999],[8,64,162,292,443,600,748,869,954,994,998,999,999,999,999],[6,61,160,290,441,599,748,869,954,994,998,998,998,998,998],[5,59,158,288,439,596,747,869,954,994,998,998,998,998,998]]],[[[8,64,162,292,442,602,748,869,954,994,998,998,998,998,998],[2,32,82,141,198,239,254,256,257,259,260,260,260,260,260],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,2,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2]],[[8,63,161,292,443,601,748,869,954,994,998,998,998,998,998],[7,62,160,291,442,600,747,868,953,994,998,998,998,998,998],[6,61,159,290,441,600,747,868,953,994,998,998,998,998,998],[4,59,157,288,439,602,747,868,953,994,998,998,998,998,998],[0,34,119,214,286,313,318,319,321,323,324,324,325,325,326]],[[8,63,162,292,443,600,747,869,954,994,998,998,998,998,998],[7,62,160,291,441,599,747,868,953,994,998,998,998,998,998],[6,61,158,289,440,599,747,868,953,993,998,998,998,998,998],[4,58,156,287,438,597,747,868,953,993,998,998,998,998,998],[2,56,155,287,437,596,747,868,953,993,998,998,998,998,998]],[[8,63,162,292,443,600,747,869,954,994,998,998,998,998,998],[7,62,160,291,442,599,747,868,953,994,998,998,998,998,998],[6,61,159,290,440,597,747,868,953,993,998,998,998,998,998],[4,58,156,286,439,596,747,868,953,993,997,998,998,998,998],[2,55,154,286,434,594,747,868,953,993,997,997,997,997,997]],[[8,64,162,293,443,600,748,869,954,994,998,998,998,998,998],[7,62,160,291,442,599,747,868,953,994,998,998,998,998,998],[6,61,159,290,441,598,747,868,953,993,998,998,998,998,998],[4,58,156,286,437,596,747,868,953,993,997,997,997,997,997],[2,55,153,284,434,596,747,868,953,993,997,997,997,997,997]]],[[[2,26,63,104,137,149,150,151,152,154,155,155,155,155,155],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,2,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2]],[[6,62,160,291,442,600,747,868,953,993,998,998,998,998,998],[5,61,158,290,441,600,747,868,953,993,997,997,997,997,997],[4,60,159,288,438,596,746,868,953,993,997,997,997,997,997],[0,-1,-2,-2,-2,-2,-1,0,2,4,5,5,5,6,6],[0,-2,-3,-3,-3,-3,-2,-1,1,3,4,4,4,5,5]],[[6,62,160,290,441,598,747,868,953,993,998,998,998,998,998],[5,59,157,289,439,597,746,868,953,993,997,997,997,997,997],[4,58,156,287,438,598,746,867,952,993,997,997,997,997,997],[2,56,155,284,436,594,746,867,952,993,997,997,997,997,997],[1,53,151,279,431,590,746,867,952,993,997,997,997,997,997]],[[7,62,160,291,441,599,747,868,953,993,998,998,998,998,998],[5,60,158,289,439,597,746,868,953,993,997,997,997,997,997],[4,58,156,287,437,597,746,867,952,993,997,997,997,997,997],[2,55,152,284,435,597,746,867,952,992,997,997,997,997,997],[1,53,151,279,434,595,746,867,952,992,997,997,997,997,997]],[[7,62,160,291,442,599,747,868,953,993,998,998,998,998,998],[5,60,158,289,440,598,746,868,953,993,997,997,997,997,997],[4,58,157,288,438,596,746,867,952,993,997,997,997,997,997],[2,55,152,283,434,594,746,867,952,992,997,997,997,997,997],[1,50,149,280,430,590,746,867,952,992,996,997,997,997,997]]],[[[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2]],[[4,58,157,287,438,598,746,867,952,992,996,997,997,997,997],[2,57,155,281,434,596,746,867,952,992,996,996,996,996,996],[0,-1,-2,-2,-2,-2,-1,0,2,3,4,5,5,6,6],[0,-2,-3,-3,-3,-3,-2,-1,1,3,4,4,4,5,5],[0,-2,-3,-3,-3,-3,-2,-1,1,3,4,4,4,5,5]],[[4,58,156,289,437,596,746,867,952,992,996,996,996,996,996],[2,55,153,285,437,594,745,866,951,992,996,996,996,996,996],[1,55,153,284,433,594,745,866,951,991,996,996,996,996,996],[0,52,145,277,431,592,745,866,951,991,996,996,996,996,996],[0,-1,-2,-2,-2,-1,0,2,4,6,8,8,9,10,10]],[[4,58,157,287,437,595,746,867,952,992,996,996,996,996,996],[3,56,153,284,436,594,745,866,951,992,996,996,996,996,996],[1,53,152,282,433,592,745,866,951,991,995,996,996,996,996],[0,48,148,278,431,589,744,866,951,991,995,995,995,995,995],[0,45,139,269,422,583,744,866,951,991,995,995,995,995,995]],[[4,59,157,288,438,596,746,867,952,992,996,996,996,996,996],[3,56,154,285,435,593,745,866,951,992,996,996,996,996,996],[1,53,151,282,432,592,745,866,951,991,995,995,995,995,995],[0,48,145,276,426,591,744,865,950,991,995,995,995,995,995],[0,43,140,272,422,584,744,865,950,991,995,995,995,995,995]]],[[[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2],[-1,-2,-3,-4,-4,-4,-3,-2,-1,1,1,2,2,2,2]],[[2,55,156,283,436,597,745,866,951,991,996,996,996,996,996],[0,-2,-3,-3,-3,-3,-2,-1,1,3,4,4,5,5,5],[0,-2,-3,-3,-3,-3,-2,-1,1,3,3,4,4,5,5],[0,-2,-3,-3,-3,-3,-2,-1,1,2,3,4,4,5,5],[0,-2,-3,-3,-3,-3,-2,-1,1,2,3,4,4,5,5]],[[2,55,154,286,435,594,745,866,951,991,995,995,995,995,995],[1,52,152,283,431,592,744,865,950,991,995,995,995,995,995],[0,49,152,279,430,591,744,865,950,990,995,995,995,995,995],[0,-1,-2,-2,-2,-1,0,2,4,6,7,8,9,9,10],[0,-2,-2,-2,-2,-1,0,1,3,6,7,8,8,9,10]],[[2,55,153,283,435,594,745,866,951,991,995,995,995,995,995],[1,52,150,281,433,593,744,865,950,990,995,995,995,995,995],[0,48,149,279,428,589,744,865,950,990,994,994,994,994,994],[0,45,145,271,422,584,743,865,950,990,994,994,994,994,994],[0,43,139,267,406,560,744,865,950,990,994,994,994,994,994]],[[2,55,154,284,435,593,745,866,951,991,995,995,995,995,995],[1,52,150,281,431,590,744,865,950,990,995,995,995,995,995],[0,49,147,277,427,592,743,865,950,990,994,994,994,994,994],[0,43,140,271,424,583,743,864,949,990,994,994,994,994,994],[0,39,135,263,417,576,743,864,949,990,994,994,994,994,994]]],[[[-1,-2,-3,-4,-4,-4,-4,-2,-1,1,1,1,2,2,2],[-1,-2,-3,-4,-4,-4,-4,-2,-1,1,1,1,2,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,2,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,2,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,2,2,2]],[[0,-2,-3,-3,-3,-3,-2,-1,1,2,3,4,4,4,5],[0,-2,-3,-3,-4,-3,-2,-1,0,2,3,4,4,4,5],[0,-2,-3,-3,-4,-3,-2,-1,0,2,3,4,4,4,5],[0,-2,-3,-3,-4,-3,-2,-1,0,2,3,4,4,4,5],[0,-2,-3,-3,-4,-3,-2,-1,0,2,3,4,4,4,5]],[[0,49,147,280,430,591,743,864,949,989,993,993,993,993,993],[0,46,143,276,425,586,742,864,948,989,993,993,993,993,993],[0,-1,-2,-2,-2,-1,0,2,4,6,7,8,8,9,10],[0,-2,-2,-3,-2,-2,0,1,3,5,7,7,8,8,9],[0,-2,-2,-3,-2,-2,0,1,3,5,7,7,8,8,9]],[[0,48,149,277,430,590,742,864,949,989,993,993,993,993,993],[0,44,143,274,426,584,742,863,948,988,993,993,993,993,993],[0,41,140,275,422,583,742,863,948,988,993,993,993,993,993],[0,-1,-1,-1,-1,1,2,4,6,9,10,11,12,13,14],[0,-1,-2,-2,-1,0,1,3,6,8,10,11,12,13,14]],[[0,49,145,282,431,589,742,864,949,989,993,993,993,993,993],[0,43,142,273,423,587,741,863,948,988,992,992,992,992,992],[0,42,138,271,418,581,741,863,947,988,992,992,992,992,992],[0,33,138,259,406,570,741,862,947,988,992,992,992,992,992],[0,-1,-1,0,1,2,4,6,9,11,13,15,16,17,19]]],[[[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,1,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,1,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,1,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,1,2,2],[-1,-2,-3,-4,-4,-4,-4,-3,-1,0,1,1,1,2,2]],[[0,-2,-3,-4,-4,-3,-2,-1,0,2,3,3,4,4,4],[0,-2,-3,-4,-4,-3,-3,-1,0,2,3,3,4,4,4],[0,-2,-3,-4,-4,-3,-3,-1,0,2,3,3,4,4,4],[0,-2,-3,-4,-4,-3,-3,-1,0,2,3,3,4,4,4],[0,-2,-3,-4,-4,-3,-3,-1,0,2,3,3,4,4,4]],[[0,44,142,276,420,585,741,862,947,988,992,992,992,992,992],[0,-1,-2,-2,-2,-1,0,1,3,5,7,7,8,8,9],[0,-2,-2,-3,-2,-2,-1,1,3,5,6,7,7,8,9],[0,-2,-2,-3,-3,-2,-1,1,3,5,6,7,7,8,9],[0,-2,-2,-3,-3,-2,-1,1,3,5,6,7,7,8,9]],[[0,46,142,272,422,585,740,862,947,987,991,991,991,991,991],[0,38,139,269,420,580,740,862,946,987,991,991,991,991,991],[0,31,132,258,411,572,740,862,946,987,991,991,991,991,991],[0,-1,-2,-2,-1,0,1,3,5,8,9,10,11,12,13],[0,-1,-2,-2,-1,0,1,3,5,8,9,10,11,12,13]],[[0,42,143,277,426,585,740,862,946,987,991,991,991,991,991],[0,39,133,270,421,581,740,861,946,986,990,990,990,990,990],[0,35,132,265,412,572,739,861,946,986,990,990,990,990,990],[0,-1,-1,0,0,2,3,6,8,11,13,14,15,16,18],[0,-1,-1,-1,0,1,3,5,7,10,12,13,15,16,17]]],[[[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1]],[[0,-2,-3,-4,-4,-4,-3,-2,0,2,3,3,3,3,4],[0,-2,-3,-4,-4,-4,-3,-2,0,2,3,3,3,3,4],[0,-2,-3,-4,-4,-4,-3,-2,0,2,3,3,3,3,4],[0,-2,-3,-4,-4,-4,-3,-2,0,2,3,3,3,3,4],[0,-2,-3,-4,-4,-4,-3,-2,0,2,3,3,3,3,4]],[[0,-2,-2,-3,-3,-2,-1,1,2,4,6,6,7,7,8],[0,-2,-3,-3,-3,-2,-1,0,2,4,5,6,6,7,7],[0,-2,-3,-3,-3,-2,-1,0,2,4,5,6,6,7,7],[0,-2,-3,-3,-3,-2,-1,0,2,4,5,6,6,7,7],[0,-2,-3,-3,-3,-2,-1,0,2,4,5,6,6,7,7]],[[0,35,135,264,414,574,738,860,945,985,989,989,989,989,989],[0,-1,-2,-2,-1,0,1,3,5,7,8,9,10,11,12],[0,-2,-2,-2,-2,-1,0,2,4,6,8,9,10,10,11],[0,-2,-2,-2,-2,-1,0,2,4,6,8,9,10,10,11],[0,-2,-2,-2,-2,-1,0,2,4,6,8,9,10,10,11]],[[0,34,133,261,414,573,737,859,944,984,988,988,988,988,988],[0,30,125,256,410,571,737,859,944,984,988,988,988,988,988],[0,0,-1,-1,0,1,3,5,7,10,11,12,14,15,16],[0,-1,-2,-2,-1,0,2,4,6,9,10,12,13,14,15],[0,-1,-2,-2,-1,0,2,4,6,9,10,12,13,14,15]]],[[[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1],[-1,-2,-3,-4,-5,-4,-4,-3,-1,0,1,1,1,1,1]],[[0,-2,-3,-4,-4,-4,-3,-2,0,1,2,3,3,3,3],[0,-2,-3,-4,-4,-4,-3,-2,0,1,2,3,3,3,3],[0,-2,-3,-4,-4,-4,-3,-2,0,1,2,3,3,3,3],[0,-2,-3,-4,-4,-4,-3,-2,0,1,2,3,3,3,3],[0,-2,-3,-4,-4,-4,-3,-2,0,1,2,3,3,3,3]],[[0,-2,-3,-3,-3,-3,-2,0,2,4,5,5,6,6,7],[0,-2,-3,-3,-3,-3,-2,0,2,4,5,5,6,6,7],[0,-2,-3,-3,-3,-3,-2,0,2,4,5,5,6,6,7],[0,-2,-3,-3,-3,-3,-2,0,2,4,5,5,6,6,7],[0,-2,-3,-3,-3,-3,-2,0,2,4,5,5,6,6,7]],[[0,-1,-2,-2,-2,-1,0,2,4,6,8,8,9,10,11],[0,-2,-2,-3,-2,-1,0,1,3,6,7,8,8,9,10],[0,-2,-2,-3,-2,-1,0,1,3,6,7,8,8,9,10],[0,-2,-2,-3,-2,-1,0,1,3,6,7,8,8,9,10],[0,-2,-2,-3,-2,-1,0,1,3,6,7,8,8,9,10]],[[0,26,123,251,408,576,736,858,943,983,987,987,987,987,987],[0,-1,-1,-1,-1,0,2,4,6,8,10,11,12,13,14],[0,-1,-2,-2,-1,0,1,3,5,8,9,10,11,12,13],[0,-1,-2,-2,-1,0,1,3,5,8,9,10,11,12,13],[0,-1,-2,-2,-1,0,1,3,5,8,9,10,11,12,13]]]]};
  const G = 9.81, KP = 600, QT = 0.030, LIFT = 0.10, TL = 1.0, TEND = 1.5;
  const liftAt = t => LIFT * 0.5 * (1 - Math.cos(Math.PI * Math.min(Math.max(t / TL, 0), 1)));
  const ctrlFor = n => Math.max(0, QT - 2 * n / KP);
  const slipAt = (tr, t) => {            // m, straight lines between the recorded 0.1 s samples
    const s = [0].concat(tr), x = Math.max(0, Math.min(t, TEND)) / 0.1, k = Math.min(Math.floor(x), 14);
    return (s[k] + (s[k + 1] - s[k]) * (x - k)) * 1e-4;
  };
  const heldRun = tr => tr[14] < 100;
  const ruleHolds = (mu, n, m) => 2 * mu * n >= m * G;

  // ---------- shared canvas drawing: front view, world y to the right, z up ----------
  // P(y, z) -> [px, py]. q = finger opening (m), rise = hand rise (m), box = box centre [y, z] or null.
  function drawRig(c, P, s, o) {
    const q = o.q, rise = o.rise || 0, fz = 0.05 + rise;
    const rect = (y0, z0, y1, z1, fill, stroke, alpha, dash) => {
      const [a, b] = P(y0, z1), [d, e] = P(y1, z0);
      c.globalAlpha = alpha == null ? 0.85 : alpha; c.fillStyle = fill; if (fill) c.fillRect(a, b, d - a, e - b);
      c.globalAlpha = 1; if (stroke) { c.strokeStyle = stroke; c.lineWidth = 1.2; c.setLineDash(dash || []); c.strokeRect(a, b, d - a, e - b); c.setLineDash([]); }
    };
    if (o.floor !== false) { const [x0, y0] = P(-1, 0), [x1] = P(1, 0); c.strokeStyle = css('--dim'); c.lineWidth = 1.5; c.beginPath(); c.moveTo(x0, y0); c.lineTo(x1, y0); c.stroke(); }
    if (o.box) rect(o.box[0] - 0.035, o.box[1] - 0.035, o.box[0] + 0.035, o.box[1] + 0.035, css('--red'), css('--red'), 0.6);
    if (o.palm !== false) rect(-0.06, 0.108 + rise, 0.06, 0.132 + rise, css('--dim'), css('--dim'), 0.35);
    if (o.ghostQ != null) {
      rect(-0.015 - o.ghostQ, fz - 0.035, -0.005 - o.ghostQ, fz + 0.035, null, css('--dim'), 1, [4, 4]);
      rect(0.005 + o.ghostQ, fz - 0.035, 0.015 + o.ghostQ, fz + 0.035, null, css('--dim'), 1, [4, 4]);
    }
    const qR = o.qR == null ? q : o.qR;
    rect(-0.015 - q, fz - 0.035, -0.005 - q, fz + 0.035, css('--orange'), css('--orange'), 0.85);
    rect(0.005 + qR, fz - 0.035, 0.015 + qR, fz + 0.035, css('--orange'), css('--orange'), 0.85);
    if (o.sites) {   // touch sites: thin boxes on the inner faces (y half 0.003, z half 0.036)
      rect(-0.008 - q, fz - 0.036, -0.002 - q, fz + 0.036, null, css('--yellow'), 1, [3, 2]);
      rect(0.002 + qR, fz - 0.036, 0.008 + qR, fz + 0.036, null, css('--yellow'), 1, [3, 2]);
    }
  }
  const mapper = (cx, floorY, s) => (y, z) => [cx + s * y, floorY - s * z];
  const label = (c, str, x, y, col, al) => text(c, str, x, y, col || css('--dim'), 11, al || 'left');

  // ======== SANDBOX 13: GRASP AND LIFT (replays MuJoCo) ========
  (() => {
    const $ = id => document.getElementById(id);
    if (!$('sandbox13')) return;
    const PX = 1500, X = y => 200 + PX * y, Y = z => 410 - PX * z;
    let si = 3, mi = 1, ui = 4, phase = 'open', q = 0.04, t = 0, raf = 0, last = 0;
    const arrowD = (x1, y1, x2, y2) => {
      const L = Math.hypot(x2 - x1, y2 - y1); if (L < 2) return '';
      const a = Math.atan2(y2 - y1, x2 - x1), h = Math.min(8, L * 0.6), f = v => v.toFixed(1);
      return `M${f(x1)} ${f(y1)}L${f(x2)} ${f(y2)}M${f(x2)} ${f(y2)}L${f(x2 - h * Math.cos(a - 0.45))} ${f(y2 - h * Math.sin(a - 0.45))}L${f(x2 - h * Math.cos(a + 0.45))} ${f(y2 - h * Math.sin(a + 0.45))}Z`;
    };
    const setRect = (id, y0, z0, y1, z1) => { const r = $(id); r.setAttribute('x', X(y0).toFixed(1)); r.setAttribute('y', Y(z1).toFixed(1)); r.setAttribute('width', (PX * (y1 - y0)).toFixed(1)); r.setAttribute('height', (PX * (z1 - z0)).toFixed(1)); };
    const setTxt = (id, v) => { const e = $(id); if (e.textContent !== v) { e.textContent = v; flash(e); } };
    function render() {
      const n = G13.meas[si], m = G13.masses[mi], mu = Math.max(G13.mus[ui], 0.6), tr = G13.slip[si][mi][ui];
      const lifting = phase === 'lift' || phase === 'done';
      const rise = lifting ? liftAt(t) : 0, slip = lifting ? slipAt(tr, t) : 0;
      const boxZ = 0.035 + Math.max(0, rise - slip), between = slip < 0.055;
      const qq = lifting && !between ? ctrlFor(G13.sets[si]) : q;
      const fz = 0.05 + rise;
      setRect('s13Box', -0.035, boxZ - 0.035, 0.035, boxZ + 0.035);
      setRect('s13Palm', -0.06, 0.108 + rise, 0.06, 0.132 + rise);
      setRect('s13FL', -0.015 - qq, fz - 0.035, -0.005 - qq, fz + 0.035);
      setRect('s13FR', 0.005 + qq, fz - 0.035, 0.015 + qq, fz + 0.035);
      const k = 12;   // px per newton
      $('s13W').setAttribute('d', arrowD(X(0), Y(boxZ), X(0), Y(boxZ) + Math.min(m * G * k, 118)));
      const gripping = (phase === 'closed' || lifting) && between;
      const zc = Y(Math.min(boxZ, fz) + 0.005);
      $('s13NL').setAttribute('d', gripping ? arrowD(X(-0.015 - qq) - n * k - 4, zc, X(-0.015 - qq) - 2, zc) : '');
      $('s13NR').setAttribute('d', gripping ? arrowD(X(0.015 + qq) + n * k + 4, zc, X(0.015 + qq) + 2, zc) : '');
      // grip each finger gives: m g / 2 while holding (in the air), else its limit mu N while sliding
      const grip = !gripping || !lifting ? 0 : (heldRun(tr) ? m * G / 2 : mu * n);
      const zf = Y(boxZ);
      $('s13FfL').setAttribute('d', grip ? arrowD(X(-0.035) + 7, zf, X(-0.035) + 7, zf - grip * k) : '');
      $('s13FfR').setAttribute('d', grip ? arrowD(X(0.035) - 7, zf, X(0.035) - 7, zf - grip * k) : '');
      const short = gripping && lifting && !heldRun(tr) ? m * G - 2 * mu * n : 0;
      $('s13Short').setAttribute('d', short > 0 ? arrowD(X(0) + 14, Y(boxZ - 0.035), X(0) + 14, Y(boxZ - 0.035) + Math.min(short * k, 60)) : '');
      // readouts
      const rule = ruleHolds(mu, n, m), held = heldRun(tr);
      setTxt('s13SetVal', G13.sets[si].toFixed(2)); setTxt('s13MassVal', m.toFixed(2)); setTxt('s13MuVal', G13.mus[ui].toFixed(1));
      setTxt('s13RN', n.toFixed(4)); setTxt('s13RF', (m * G / 2).toFixed(4)); setTxt('s13RA', (mu * n).toFixed(4));
      setTxt('s13RP', rule ? 'holds' : 'slips');
      setTxt('s13RR', (held ? 'held, ' : 'slipped, ') + (tr[14] / 10).toFixed(1) + ' mm at 1.5 s');
      $('s13RS').textContent = (slip * 1000).toFixed(1);
      $('s13TimeText').textContent = lifting ? 'lift time: ' + t.toFixed(2) + ' s' : (phase === 'open' ? 'fingers open 0.040 m' : 'closed, squeezing');
      $('s13Verdict').textContent = 'rule: ' + (rule ? 'holds' : 'slips') + ' | MuJoCo: ' + (held ? 'held' : 'slipped');
      $('s13Verdict').setAttribute('fill', rule === held ? 'var(--teal)' : 'var(--orange)');
    }
    const stop = () => { if (raf) cancelAnimationFrame(raf); raf = 0; };
    function animate(dur, onTick, onDone) {
      stop();
      if (RM) { onTick(1); onDone(); render(); return; }
      const t0 = performance.now();
      const loop = now => { const f = Math.min((now - t0) / 1000 / dur, 1); onTick(f); render(); if (f < 1) raf = requestAnimationFrame(loop); else { raf = 0; onDone(); render(); } };
      raf = requestAnimationFrame(loop);
    }
    function close(then) {
      if (phase !== 'open') { if (then) then(); return; }
      phase = 'closing';
      animate(0.4, f => { q = 0.04 - 0.01 * f; }, () => { phase = 'closed'; q = QT; if (then) then(); });
    }
    function lift() {
      close(() => { phase = 'lift'; t = 0; animate(TEND, f => { t = TEND * f; }, () => { phase = 'done'; t = TEND; }); });
    }
    function reset() { stop(); phase = 'open'; q = 0.04; t = 0; render(); }
    $('s13Close').addEventListener('click', () => { if (phase === 'done') reset(); close(); });
    $('s13Lift').addEventListener('click', () => { if (phase === 'done' || phase === 'lift') { stop(); phase = 'closed'; q = QT; } lift(); });
    $('s13Reset').addEventListener('click', reset);
    [['s13Set', v => { si = v; }], ['s13Mass', v => { mi = v; }], ['s13Mu', v => { ui = v; }]].forEach(([id, f]) =>
      $(id).addEventListener('input', e => { f(+e.target.value); reset(); }));
    render();
  })();

  // ======== LESSON 13 EXPLAINER ========
  (() => {
    // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
    const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson13').classList.contains('active')) fn(e); }, opts);
    const $ = id => document.getElementById(id);
    const setRo = (id, v) => { const e = $(id).querySelector('.val'); if (e.textContent !== v) { e.textContent = v; flash($(id)); } };
    const pick = (cls, k, attr) => document.querySelectorAll('#lesson13 .' + cls).forEach(b => b.classList.toggle('primary', b.dataset[attr] === String(k)));

    /* ---------- Section 1: free-body diagram (textbook rule only) ---------- */
    (() => {
      const cv = $('l13cvA'); let g = fit(cv, 0.42);
      function draw() {
        const { ctx: c, w, h } = g, N = +$('l13aN').value, m = +$('l13aM').value, mu = +$('l13aU').value;
        c.clearRect(0, 0, w, h);
        const s = Math.min(0.3 * h, 0.22 * w) / 0.07, by0 = h * 0.42, P = mapper(w / 2, by0 + 0.035 * s, s);
        const k = Math.min(h * 0.08, w * 0.045), cap = h * 0.38;
        drawRig(c, P, s, { q: 0.03, rise: -0.015, box: [0, 0.035], floor: false, palm: false });
        const [bx, by] = P(0, 0.035), half = 0.035 * s, need = m * G / 2, lim = mu * N;
        const wl = Math.min(m * G * k, h - by - 8);
        arrow(c, bx, by, bx, by + wl, css('--orange'), 3, 9);
        label(c, 'm g = ' + (m * G).toFixed(2) + ' N' + (m * G * k > wl + 1 ? ' (arrow cut)' : ''), bx + 10, by + wl - 4, css('--orange'));
        const fxL = bx - half - 0.01 * s, fxR = bx + half + 0.01 * s, nl = Math.min(N * k, fxL - 8);
        if (N > 0) {
          arrow(c, fxL - nl - 2, by + half * 0.6, fxL - 2, by + half * 0.6, css('--blue'), 3, 8);
          arrow(c, fxR + nl + 2, by + half * 0.6, fxR + 2, by + half * 0.6, css('--blue'), 3, 8);
          label(c, 'N = ' + N.toFixed(2) + ' N', fxL - 6, by + half * 0.6 + 16, css('--blue'), 'right');
        }
        const used = Math.min(need, lim), ll = Math.min(lim * k, cap);
        [bx - half + 8, bx + half - 8].forEach(x => {
          c.setLineDash([4, 4]); c.strokeStyle = css('--teal'); c.lineWidth = 1.5;
          c.strokeRect(x - 5, by - ll, 10, ll); c.setLineDash([]);
          if (used > 0) arrow(c, x, by, x, by - Math.min(used * k, cap), css('--teal'), 3, 8);
        });
        label(c, 'limit μ N = ' + lim.toFixed(2) + ' N' + (lim * k > cap ? ' (arrow cut)' : ''), fxR + 8, by - ll + 6, css('--teal'));
        label(c, 'grip used ' + used.toFixed(2) + ' N', fxR + 8, by - ll + 22, css('--teal'));
        if (need > lim) { const sl = Math.min((m * G - 2 * lim) * k, h - by - half - 8); arrow(c, bx - 22, by + half, bx - 22, by + half + sl, css('--red'), 2.5, 8); label(c, 'shortfall ' + (m * G - 2 * lim).toFixed(2) + ' N', bx - 30, by + half + 14, css('--red'), 'right'); }
        $('l13aNo').textContent = N.toFixed(2) + ' N'; $('l13aMo').textContent = m.toFixed(2) + ' kg'; $('l13aUo').textContent = mu.toFixed(2);
        setRo('l13aW', (m * G).toFixed(4)); setRo('l13aF', need.toFixed(4)); setRo('l13aL', lim.toFixed(4));
        setRo('l13aV', need <= lim ? 'holds' : 'slips');
      }
      ['l13aN', 'l13aM', 'l13aU'].forEach(id => $(id).addEventListener('input', draw));
      addEventListener('resize', () => { g = fit(cv, 0.42); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 2: labelled model ---------- */
    (() => {
      const cv = $('l13cvB'); let g = fit(cv, 0.62);
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const s = Math.min(w / 0.42, h / 0.165), P = mapper(w / 2, h - 18, s);
        drawRig(c, P, s, { q: 0.04, box: [0, 0.035], sites: true });
        const lab = (y, z, ty, tz, str, col) => { const [x1, y1] = P(y, z), [x2, y2] = P(ty, tz); c.strokeStyle = css('--dim'); c.lineWidth = 1; c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke(); label(c, str, x2 + (ty < y ? -4 : 4), y2, col || css('--bright'), ty < y ? 'right' : 'left'); };
        lab(0.06, 0.125, 0.075, 0.145, 'hand, lift (slide z)');
        lab(-0.055, 0.075, -0.075, 0.1, 'grip_L: -y', css('--orange'));
        lab(0.055, 0.075, 0.075, 0.1, 'grip_R: +y', css('--orange'));
        lab(-0.045, 0.03, -0.075, 0.04, 'touch_L_site', css('--yellow'));
        lab(0.025, 0.02, 0.075, 0.03, 'box 0.25 kg', css('--red'));
        lab(-0.09, 0, -0.1, 0.012, 'floor');
        const [ax, ay] = P(-0.055, 0.05), [bx2] = P(-0.075, 0.05); arrow(c, ax, ay, bx2, ay, css('--orange'), 2, 6);
        const [cx2, cy2] = P(0.055, 0.05), [dx] = P(0.075, 0.05); arrow(c, cx2, cy2, dx, cy2, css('--orange'), 2, 6);
        label(c, '7 cm', P(0, 0.035)[0], P(0, 0.035)[1], css('--bright'), 'center');
      }
      addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 3: coupling (numbers from part B of the script) ---------- */
    (() => {
      const cv = $('l13cvC'); let g = fit(cv, 0.7), vk = 0, tk = 0;
      const V = [  // [grip_L, grip_R, ten_length] at start, after 1 step, after 0.5 s (mj_forward before reading)
        [[0.04, 0.04, '0'], [0.039623, 0.039691, '-6.86e-05'], [0.020000, 0.020001, '-8.35e-07']],
        [[0.04, 0.04, '0'], [0.039623, 0.039691, '-6.86e-05'], [0.020000, 0.020001, '-8.35e-07']],
        [[0.04, 0.04, '0'], [0.039314, 0.040000, '-6.86e-04'], [0.020000, 0.040000, '-2.00e-02']]];
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const [qL, qR, tl] = V[vk][tk], s = Math.min(w / 0.17, h / 0.13), P = mapper(w / 2, h - 0.03 * s, s);
        drawRig(c, P, s, { q: qL, qR, ghostQ: tk ? 0.04 : null, floor: false, palm: false, rise: -0.04 });
        const [tx, ty] = P(0, 0.02);
        label(c, 'target 0.020', tx, ty, css('--dim'), 'center');
        [[-0.005 - 0.02, 1], [0.005 + 0.02, 1]].forEach(([y]) => { const [x1, y1] = P(y, -0.01), [, y2] = P(y, 0.07); c.setLineDash([2, 4]); c.strokeStyle = css('--teal'); c.lineWidth = 1; c.beginPath(); c.moveTo(x1, y1); c.lineTo(x1, y2); c.stroke(); c.setLineDash([]); });
        label(c, ['tendon + equality', 'joint equality', 'no coupling'][vk] + ', ' + ['start', 'after 1 step', 'after 0.5 s'][tk], 8, 12, css('--bright'));
        setRo('l13cL', qL.toFixed(6)); setRo('l13cR', qR.toFixed(6)); setRo('l13cT', tl);
      }
      document.querySelectorAll('#lesson13 .l13cv').forEach(b => b.addEventListener('click', () => { vk = +b.dataset.k; pick('l13cv', vk, 'k'); draw(); }));
      document.querySelectorAll('#lesson13 .l13ct').forEach(b => b.addEventListener('click', () => { tk = +b.dataset.t; pick('l13ct', tk, 't'); draw(); }));
      addEventListener('resize', () => { g = fit(cv, 0.7); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 4: the squeeze, on the floor vs held (numbers from part C) ---------- */
    (() => {
      const cv = $('l13cvD'); let g = fit(cv, 0.9), k = 0;
      const S = [
        { tL: '1.5629', tR: '0.2607', nL: 1.5629, nR: 0.2607, vL: -0.5036, vR: -0.2371, floor: 3.1931, act: '-1.8227', viol: '-3.74e-04', qL: 0.029704, qR: 0.030078, rise: 0,
          txt: 'On the floor: lopsided squeeze. The hand is still settling 1.71 mm onto its lift spring, so the fingers drag the box down a little (0.5036 N and 0.2371 N). The floor pushes 3.1931 N, more than the 2.4525 N weight.' },
        { tL: '0.8533', tR: '0.8535', nL: 0.8533, nR: 0.8535, vL: 1.2463, vR: 1.2062, floor: 0, act: '-1.7070', viol: '-7.66e-04', qL: 0.029512, qR: 0.030278, rise: 0.1,
          txt: 'Held in the air: equal squeezes. The two upward grips, 1.2463 N + 1.2062 N, add up to 2.4525 N, exactly the weight.' }];
      function draw() {
        const { ctx: c, w, h } = g, d = S[k]; c.clearRect(0, 0, w, h);
        const s = Math.min(w / 0.2, h / 0.2), P = mapper(w / 2, h * 0.62 + (k ? 0.1 * s : 0), s), kk = h / 9;
        const boxY = (d.qR - d.qL) / 2;          // the box sits where the two faces meet
        drawRig(c, P, s, { q: d.qL, qR: d.qR, rise: d.rise, box: [boxY, 0.035 + d.rise], floor: !k, palm: false });
        const [bx, by] = P(boxY, 0.035 + d.rise), half = 0.035 * s, fw = 0.01 * s;
        arrow(c, bx, by, bx, by + 2.4525 * kk, css('--orange'), 3, 8); label(c, 'm g 2.4525 N', bx + 8, by + 2.4525 * kk + 2, css('--orange'));
        arrow(c, bx - half - fw - d.nL * kk - 4, by - 18, bx - half - fw - 2, by - 18, css('--blue'), 3, 8);
        arrow(c, bx + half + fw + d.nR * kk + 4, by - 18, bx + half + fw + 2, by - 18, css('--blue'), 3, 8);
        label(c, d.tL + ' N', bx - half - fw - 4, by - 32, css('--blue'), 'right'); label(c, d.tR + ' N', bx + half + fw + 4, by - 32, css('--blue'));
        arrow(c, bx - half + 6, by, bx - half + 6, by - d.vL * kk, css('--teal'), 3, 7);
        arrow(c, bx + half - 6, by, bx + half - 6, by - d.vR * kk, css('--teal'), 3, 7);
        if (d.floor) { const [, fy] = P(0, 0); arrow(c, bx - 16, fy + d.floor * kk, bx - 16, fy, css('--purple'), 3, 8); label(c, 'floor 3.1931 N', bx - 22, fy + 14, css('--purple'), 'right'); }
        setRo('l13dTL', d.tL); setRo('l13dTR', d.tR); setRo('l13dA', d.act); setRo('l13dQ', d.viol);
        $('l13dTxt').textContent = d.txt;
      }
      document.querySelectorAll('#lesson13 .l13dw').forEach(b => b.addEventListener('click', () => { k = +b.dataset.k; pick('l13dw', k, 'k'); draw(); }));
      addEventListener('resize', () => { g = fit(cv, 0.9); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 5: the recorded grid for one mu ---------- */
    (() => {
      const cv = $('l13cvE'); let g = fit(cv, 0.6), ui = 4;
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const L = 58, T = 26, B = 40, R = 8, nc = G13.sets.length, nr = G13.masses.length;
        const cw = (w - L - R) / nc, ch = (h - T - B) / nr, mu = G13.mus[ui];
        let held = 0, agree = 0;
        for (let i = 0; i < nc; i++) for (let j = 0; j < nr; j++) {
          const tr = G13.slip[i][j][ui], ok = heldRun(tr), rule = ruleHolds(mu, G13.meas[i], G13.masses[j]);
          held += ok; agree += ok === rule;
          const x = L + i * cw, y = T + j * ch;
          c.globalAlpha = 0.55; c.fillStyle = ok ? css('--teal') : css('--orange'); c.fillRect(x + 1, y + 1, cw - 2, ch - 2); c.globalAlpha = 1;
          if (cw > 34) text(c, (tr[14] / 10).toFixed(1), x + cw / 2, y + ch / 2, css('--bright'), 10, 'center');
          if (ok !== rule) { c.strokeStyle = css('--red'); c.lineWidth = 2.5; c.beginPath(); c.moveTo(x + 4, y + 4); c.lineTo(x + cw - 4, y + ch - 4); c.moveTo(x + cw - 4, y + 4); c.lineTo(x + 4, y + ch - 4); c.stroke(); }
        }
        // rule edge: for each mass, where the measured squeeze reaches m g / (2 mu)
        const xOf = need => {
          const M = G13.meas; if (need <= M[0]) return L + cw * 0.5 * need / M[0];
          for (let i = 1; i < nc; i++) if (need <= M[i]) return L + cw * (i - 0.5 + (need - M[i - 1]) / (M[i] - M[i - 1]));
          return w - R;
        };
        c.strokeStyle = css('--bright'); c.lineWidth = 2; c.beginPath();
        G13.masses.forEach((m, j) => { const x = xOf(m * G / (2 * mu)); if (j === 0) c.moveTo(x, T); c.lineTo(x, T + j * ch + ch / 2); if (j === nr - 1) c.lineTo(x, T + nr * ch); });
        c.stroke();
        G13.masses.forEach((m, j) => text(c, m.toFixed(2) + ' kg', L - 6, T + j * ch + ch / 2, css('--dim'), 10, 'right'));
        G13.sets.forEach((s, i) => { if (cw > 26 || i % 2 === 1) text(c, String(s), L + i * cw + cw / 2, h - B + 12, css('--dim'), 10, 'center'); });
        text(c, 'setting per finger (N), cell = slip (mm)', L, h - 10, css('--dim'), 10);
        text(c, 'μ = ' + mu.toFixed(1) + ', line: 2 μ N = m g', L, 12, css('--bright'), 11);
        setRo('l13eH', String(held)); setRo('l13eA', String(agree));
      }
      document.querySelectorAll('#lesson13 .l13em').forEach(b => b.addEventListener('click', () => { ui = +b.dataset.k; pick('l13em', ui, 'k'); draw(); }));
      addEventListener('resize', () => { g = fit(cv, 0.6); draw(); }); addEventListener('themechange', draw);
      draw();
    })();

    /* ---------- Section 6: touch sensor vs contact pairs (numbers from part E) ---------- */
    (() => {
      const cv = $('l13cvF'); let g = fit(cv, 0.85), k = 0;
      const OUT = [
        'pairs in data.contact:\nfloor-finger_L: 4 contacts, 24.104 N\nfloor-finger_R: 4 contacts, 24.104 N\nfloor-box:      4 contacts,  2.452 N\nboth fingers touch the box? False',
        'pairs in data.contact:\nfinger_L-box: 4 contacts, 3.473 N\nfinger_R-box: 4 contacts, 3.474 N\nboth fingers touch the box? True'];
      function draw() {
        const { ctx: c, w, h } = g; c.clearRect(0, 0, w, h);
        const s = Math.min(w / 0.17, h / 0.25), P = mapper(w / 2, h - 16, s);
        const dot = (y, z, inSite) => { const [x, yy] = P(y, z); c.fillStyle = css('--red'); c.beginPath(); c.arc(x, yy, 4, 0, 7); c.fill(); if (inSite) { c.strokeStyle = css('--yellow'); c.lineWidth = 2; c.beginPath(); c.arc(x, yy, 8, 0, 7); c.stroke(); } };
        if (k === 0) {
          const rise = -0.01589;
          drawRig(c, P, s, { q: 0.04, rise, box: [0, 0.035], sites: true });
          [-1, 1].forEach(sg => { dot(sg * 0.045, 0, true); dot(sg * 0.055, 0, false); dot(sg * 0.035, 0, false); });
          label(c, 'open, pressed on floor', 8, 12, css('--bright'));
        } else {
          const rise = 0.1;
          drawRig(c, P, s, { q: 0.03, rise, box: [0, 0.035 + rise], sites: true });
          [-1, 1].forEach(sg => { dot(sg * 0.035, 0.015 + rise, true); dot(sg * 0.035, 0.07 + rise, true); });
          label(c, 'closed on box, lifted', 8, 12, css('--bright'));
        }
        label(c, 'dot = contact, ring = in a site', 8, 28, css('--dim'));
        setRo('l13fTL', k ? '3.473' : '12.052'); setRo('l13fTR', k ? '3.474' : '12.052');
        $('l13fOut').textContent = OUT[k];
      }
      document.querySelectorAll('#lesson13 .l13fs').forEach(b => b.addEventListener('click', () => { k = +b.dataset.k; pick('l13fs', k, 'k'); draw(); }));
      addEventListener('resize', () => { g = fit(cv, 0.85); draw(); }); addEventListener('themechange', draw);
      draw();
    })();
  })();
})();
