  #include <math.h>
  #include <string.h>
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  #include <Preferences.h>
Preferences prefs;

const uint32_t MAP_VERSION = 0x20260306;
bool enableLoadFromFlash = true;   // muốn test map lưu thì để true

bool savedMapAvailable = false;
bool fastRunReady = false;

char plannedPath[256];
int plannedLen = 0;

const float CELL_LEN_CM = 18.0f;


  // ======= 1. ENUM =======
  enum Heading { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
  const Heading FAST_START_HEADING = EAST;   // chỉnh lại nếu lúc fast-run robot không quay mặt EAST

  struct DistMM {
    int f, l, r;   // front, left, right
    bool f_ok, l_ok, r_ok;
  };

 // =============== ENCODER ===============
#define C1_LEFT 18
#define C2_LEFT 19
#define C1_RIGHT 4
#define C2_RIGHT 5

// =============== TB6612 ===============

#define PWMA 25
#define AIN2 27
#define AIN1 26
#define STBY 14
#define BIN1 12
#define BIN2 13
#define PWMB 23

// =============== BUTTON ================
#define button_1 16
#define button_2 17

// =============== IR SENSOR ===============
#define IR_1 36
#define IR_2 39
#define IR_3 34
#define IR_4 35
#define IR_5 32

  //--------------------------------------khởi tạo map--------------------------------------------

  const int MAZE_SIZE = 33;

  const int maze_default[MAZE_SIZE][MAZE_SIZE] = {
    { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 }

  };

  int maze[MAZE_SIZE][MAZE_SIZE];

  // -----------------------------khai báo giải mê cung và vị trí robot---------------------------------


  int dist[MAZE_SIZE][MAZE_SIZE];
  int dx[4] = {-2, 0, 2, 0};
  int dy[4] = {0, 2, 0, -2};

  // =================== tự tạo queue====================
  struct Node {
    int x, y;
  };

  Node q[600];   // Hàng đợi
  int q_front = 0;
  int q_back = 0;

  void q_reset() {
    q_front = 0;
    q_back = 0;
  }

  bool q_empty() {
    return q_front == q_back;
  }

  void q_push(int x, int y) {
    q[q_back].x = x;
    q[q_back].y = y;
    q_back = (q_back + 1) % 600;
  }

  Node q_pop() {
    Node n = q[q_front];
    q_front = (q_front + 1) % 600;
    return n;
  }

  // -=========================================================
  int startDirection = 0;

  //đích số 1
  int goal_x1 = 1;
  int goal_y1 = 1;

  // đích số 2
  int goal_x2 = 15;
  int goal_y2 = 15;

  // // đích số 2
  // int goal_x2 = 5;
  // int goal_y2 = 3;

  // ======= 2. PROTOTYPES =======
  bool wall_front();
  bool wall_left();
  bool wall_right();

  void update_frontWall(int cx, int cy, Heading h);
  void update_leftWall(int cx, int cy, Heading h);
  void update_rightWall(int cx, int cy, Heading h);

  // ======= 3. BIẾN =======
  Heading heading = EAST;

  // ======= 4. HÀM SENSOR =======
  DistMM read_sensor() {
    DistMM d;
    
    // Lấy trung bình đọc cảm biến để giảm nhiễu
    long sum1 = 0, sum3 = 0, sum5 = 0;
    for(int i = 0; i < 5; i++) {
      sum1 += analogRead(IR_1);
      sum3 += analogRead(IR_3);
      sum5 += analogRead(IR_5);
    }
    
    // Chuyển đổi sang cm bằng hàm tuyến tính của bạn
    float dist_l_cm = getDistance(sum1 / 5);
    float dist_f_cm = getDistance(sum3 / 5);
    float dist_r_cm = getDistance(sum5 / 5);
    
    // Chuyển kết quả sang mm cho tương thích với logic cũ (laser trả về mm)
    d.l = (int)(dist_l_cm * 10);
    d.f = (int)(dist_f_cm * 10);
    d.r = (int)(dist_r_cm * 10);
    
    // Đánh dấu tính hợp lệ của cảm biến (ví dụ < 30cm được coi là OK)
    d.l_ok = (dist_l_cm > 0 && dist_l_cm < 30);
    d.f_ok = (dist_f_cm > 0 && dist_f_cm < 30);
    d.r_ok = (dist_r_cm > 0 && dist_r_cm < 30);
    
    return d;
  }

  bool wall_front() {
    DistMM d = read_sensor();
    return d.f_ok && d.f < 160;
  }

  bool wall_left() {
    DistMM d = read_sensor();
    return d.l_ok && d.l < 160;
  }

  bool wall_right() {
    DistMM d = read_sensor();
    return d.r_ok && d.r < 160;
  }

  // ======= 5. HÀM CẬP NHẬT WALL =======
  void update_frontWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;

      if(h==NORTH) maze[mx-1][my] = 3;
      else if(h==SOUTH) maze[mx+1][my] = 3;
      else if(h==EAST)  maze[mx][my+1] = 2;
      else if(h==WEST)  maze[mx][my-1] = 2;
  }

  void update_leftWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;
      Heading L = (Heading)((h+3)%4);

      if(L==NORTH) maze[mx-1][my] = 3;
      else if(L==SOUTH) maze[mx+1][my] = 3;
      else if(L==EAST)  maze[mx][my+1] = 2;
      else if(L==WEST)  maze[mx][my-1] = 2;
  }

  void update_rightWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;
      Heading R = (Heading)((h+1)%4);

      if(R==NORTH) maze[mx-1][my] = 3;
      else if(R==SOUTH) maze[mx+1][my] = 3;
      else if(R==EAST)  maze[mx][my+1] = 2;
      else if(R==WEST)  maze[mx][my-1] = 2;
  }

  void clear_frontWall(int cx, int cy, Heading h) {
  int mx = cx*2-1, my = cy*2-1;

  if (h == NORTH) {
    if (maze[mx-1][my] == 5) maze[mx-1][my] = 0;   // chỉ xóa ??? -> open
  }
  else if (h == SOUTH) {
    if (maze[mx+1][my] == 5) maze[mx+1][my] = 0;
  }
  else if (h == EAST) {
    if (maze[mx][my+1] == 4) maze[mx][my+1] = 0;   // chỉ xóa ? -> open
  }
  else if (h == WEST) {
    if (maze[mx][my-1] == 4) maze[mx][my-1] = 0;
  }
}

  void clear_leftWall(int cx, int cy, Heading h) {
  int mx = cx*2-1, my = cy*2-1;
  Heading L = (Heading)((h+3)%4);

  if (L == NORTH) {
    if (maze[mx-1][my] == 5) maze[mx-1][my] = 0;
  }
  else if (L == SOUTH) {
    if (maze[mx+1][my] == 5) maze[mx+1][my] = 0;
  }
  else if (L == EAST) {
    if (maze[mx][my+1] == 4) maze[mx][my+1] = 0;
  }
  else if (L == WEST) {
    if (maze[mx][my-1] == 4) maze[mx][my-1] = 0;
  }
}

  void clear_rightWall(int cx, int cy, Heading h) {
  int mx = cx*2-1, my = cy*2-1;
  Heading R = (Heading)((h+1)%4);

  if (R == NORTH) {
    if (maze[mx-1][my] == 5) maze[mx-1][my] = 0;
  }
  else if (R == SOUTH) {
    if (maze[mx+1][my] == 5) maze[mx+1][my] = 0;
  }
  else if (R == EAST) {
    if (maze[mx][my+1] == 4) maze[mx][my+1] = 0;
  }
  else if (R == WEST) {
    if (maze[mx][my-1] == 4) maze[mx][my-1] = 0;
  }
}

  // ======== hàm phát hiện có tường và vẽ==========
  void detect_update_maze(int cx, int cy, Heading h) {
      if (wall_front()) update_frontWall(cx, cy, h); else clear_frontWall(cx, cy, h);
      if (wall_left())  update_leftWall(cx, cy, h); else clear_leftWall(cx, cy, h);
      if (wall_right()) update_rightWall(cx, cy, h); else clear_rightWall(cx, cy, h);
  }


  bool canMove(int x, int y, int nx, int ny) {

      // kiểm tra biên
      if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE)
          return false;

      // Tường nằm giữa ô hiện tại và ô đích
      int wallX = (x + nx) / 2;
      int wallY = (y + ny) / 2;

      // Nếu giữa 2 ô có tường (2,3) hoặc tường UNKNOWN (4,5) → KHÔNG ĐI
      int w = maze[wallX][wallY];
      if (w == 2 || w == 3)
          return false;

      // Ô đích phải là ô trống hợp lệ
      if (maze[nx][ny] != 0)
          return false;

      return true;
  }

  // floodfill từ vị trí cần đổ khi chưa có tường
  void floodFill(int startX, int startY) {
      memset(dist, -1, sizeof(dist));

      q_reset();
      dist[startX][startY] = 0;
      q_push(startX, startY);

      while (!q_empty()) {
          Node cur = q_pop();
          int x = cur.x;
          int y = cur.y;

          for (int d = 0; d < 4; d++) {
              int nx = x + dx[d];
              int ny = y + dy[d];

              if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE)
                continue;

              // if (maze[nx][ny] == 4 || maze[nx][ny] == 5)
              //   continue;

              if (!canMove(x, y, nx, ny))
                continue;

              if (dist[nx][ny] == -1) {
                  dist[nx][ny] = dist[x][y] + 1;
                  q_push(nx, ny);
              }
          }
      }
  }


  Heading chooseBestDirection(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;

      int best = 9999;
      Heading bestDir = h;

      for(int d=0;d<4;d++){
          int nx = mx + dx[d];
          int ny = my + dy[d];

          if(nx<0||ny<0||nx>=MAZE_SIZE||ny>=MAZE_SIZE) continue;
          if(!canMove(mx,my,nx,ny)) continue;

          // if(dist[nx][ny] < best && dist[nx][ny] >= 0){
          //     best = dist[nx][ny];
          //     bestDir = (Heading)d;
          // }
          // Chỉ xét các ô có giá trị dist hợp lệ
        if(dist[nx][ny] >= 0) {
            
            // Trường hợp 1: Tìm thấy đường ngắn hơn hẳn
            if(dist[nx][ny] < best){
                best = dist[nx][ny];
                bestDir = (Heading)d;
            }
            // Trường hợp 2: Khoảng cách BẰNG nhau, nhưng đây là hướng đi thẳng
            else if(dist[nx][ny] == best && (Heading)d == h){
                bestDir = (Heading)d;
            }
        }
      }
      return bestDir;
  }

  void printMazeBT() {
      for (int i = 0; i < MAZE_SIZE; i++) {
          for (int j = 0; j < MAZE_SIZE; j++) {
              int cell = maze[i][j];

              if (cell == 1)        Serial.print("o");
              else if (cell == 2)   Serial.print("|");
              else if (cell == 3)   Serial.print("---");
              else if (cell == 4)   Serial.print("?");
              else if (cell == 5)   Serial.print("???");
              else if (cell == 0) {
                  int d = dist[i][j];
                  if (d < 0) Serial.print("   ");   // chưa reachable
                  else if (d < 10) {
                      Serial.print(" ");
                      Serial.print(d);
                      Serial.print(" ");
                  } else {
                      Serial.print(d);
                      Serial.print(" ");
                  }
              }
          }
          Serial.println();
      }
      Serial.println();
  }

// void printMazeBT() {
//       for (int i = 0; i < MAZE_SIZE; i++) {
//           for (int j = 0; j < MAZE_SIZE; j++) {
//               int cell = maze[i][j];

//               if (cell == 1)        SerialBT.print("o");
//               else if (cell == 2)   SerialBT.print("|");
//               else if (cell == 3)   SerialBT.print("---");
//               else if (cell == 4)   SerialBT.print("?");
//               else if (cell == 5)   SerialBT.print("???");
//               else if (cell == 0) {
//                   int d = dist[i][j];
//                   if (d < 0) SerialBT.print("   ");   // chưa reachable
//                   else if (d < 10) {
//                       SerialBT.print(" ");
//                       SerialBT.print(d);
//                       SerialBT.print(" ");
//                   } else {
//                       SerialBT.print(d);
//                       SerialBT.print(" ");
//                   }
//               }
//           }
//           SerialBT.println();
//       }
//       SerialBT.println();
//   }

  // số xung ở vị trí ban đầu
  volatile int encoder1_val = 0;
  volatile int encoder2_val = 0;

  // biến đánh dấu đến đích và chuyển vùng floodfill
  int m =0;
  
  // vị trí ban đầu robot
  int cx = 1, cy = 1;     // vị trí cell logic
  int count_goal = 0;
  int finish = 0;

  // -------------------------------------------------------------------------------------------
  // ----------- motor và encoder ------------
  void IRAM_ATTR encoder1() {
  int A = digitalRead(C1_LEFT);
  int B = digitalRead(C2_LEFT);

  if(A != B) {
    encoder1_val --;
  }
  else {
    encoder1_val ++;
  }
}

void IRAM_ATTR encoder2() {
  int A = digitalRead(C1_RIGHT);
  int B = digitalRead(C2_RIGHT);

  if(A != B) {
    encoder2_val ++;
  }
  else {
    encoder2_val --;
  }
}

void motor_run(double speed_left, double speed_right) {
  digitalWrite(STBY, HIGH);

  if(speed_left > 0) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed_left);
  }
  else {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, abs(speed_left));
  }

  if(speed_right > 0) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed_right);
  }

  else {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, abs(speed_right));
  }
}


// void motor_stop() {
//   // 1. Phanh cứng (Short Brake) động cơ trước
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, HIGH);
//   analogWrite(PWMA, 0);
//   analogWrite(PWMB, 0);
  
//   // 2. Chờ một tẹo cho xe khựng lại hẳn
//   delay(10); 

//   // 3. Đưa IC về chế độ ngủ (Tắt hoàn toàn tiếng kêu)
//   digitalWrite(STBY, LOW);
// }

  void motor_stop() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(STBY, LOW);
  }

  // ---------------------------------------------------------------------------------------

  // Đọc tổng xung quay (ổn định hơn dùng 1 bánh)
  static inline int spinCount() {
    noInterrupts();
    int e1 = encoder1_val;
    int e2 = encoder2_val;
    interrupts();
    return (abs(e1) + abs(e2)) / 2;
  }

  // Reset 2 encoder
  static inline void reset_encoder() {
    noInterrupts();
    encoder1_val = 0;
    encoder2_val = 0;
    interrupts();
  }

  float kp;
float ki;
float kd;

float e;
float eprev = 0;
float eintegral = 0;

// BẢNG DỮ LIỆU THỰC TẾ (SỬ DỤNG CHUNG CHO CẢ 2 CẢM BIẾN)
// Nếu 2 cảm biến sai lệch nhiều, bạn có thể cần tạo bảng riêng cho từng cái.
int analogValues[] = {3800, 2500, 1500,  900,  600,  400,  200}; 
int distValues[]   = {   2,    4,    6,    8,   10,   15,   20}; 
int numReadings = 7; 

// Hàm nội suy tuyến tính
float getDistance(int rawVal) {
  if (rawVal > analogValues[0]) return distValues[0];
  if (rawVal < analogValues[numReadings-1]) return -1.0; 

  for (int i = 0; i < numReadings - 1; i++) {
    if (rawVal <= analogValues[i] && rawVal >= analogValues[i+1]) {
      int x = rawVal;
      int x0 = analogValues[i];
      int x1 = analogValues[i+1];
      int y0 = distValues[i];
      int y1 = distValues[i+1];

      return y0 + (float)(x - x0) * (float)(y1 - y0) / (float)(x1 - x0);
    }
  }
  return 0;
}

float SPEED_L; 
float SPEED_R;

float speed_l;
float speed_r;

  // -------------------------------------------xoay góc----------------------------------------------
  // Hồ sơ vận tốc + phanh chủ động (blocking ~1–2s tùy tune)
  // ===== TURN V3 – MƯỢT NHẤT, CHUẨN MICROMOUSE =====

  #define TURN_90_COUNTS    430        // calibrate, lucs ddaauf laf 830
  #define TURN_MAX_PWM      190        // lực tối đa khi quay
  #define TURN_MIN_PWM      60        // vượt ma sát
  #define TURN_ACCEL      20000.0f     // gia tốc
  #define TURN_DECEL      22000.0f     // giảm tốc
  #define TURN_MAX_SPEED   8500.0f     // tốc độ đỉnh (counts/s)
  #define TURN_SETTLE_MS     20        // dừng ổn định

  static void turn_90_profiled(int dir) {
    reset_encoder();

    unsigned long last_us = micros();
    float speed = 0;  // tốc độ hiện tại (counts/s)

    while (true) {
        // dt
        unsigned long now = micros();
        float dt = (now - last_us) * 1e-6f;
        if (dt <= 0) dt = 0.001f;
        last_us = now;

        // tiến độ
        int C = spinCount();
        int rem = TURN_90_COUNTS - C;
        Serial.println(rem);

        if (rem <= 0) break;  // hoàn thành

        // tăng/giảm tốc S-curve
        float v_accel = speed + TURN_ACCEL * dt;
        float v_brake = sqrtf(fmaxf(0, 2.0f * TURN_DECEL * rem));
        float v_cmd = fminf(TURN_MAX_SPEED, fminf(v_accel, v_brake));
        speed = v_cmd;

        // map tốc độ -> PWM
        int pwm = TURN_MIN_PWM + (int)((TURN_MAX_PWM - TURN_MIN_PWM) * (v_cmd / TURN_MAX_SPEED));
        pwm = constrain(pwm, TURN_MIN_PWM, TURN_MAX_PWM);

        // điều khiển quay tại chỗ bằng vi sai encoder để không lệch góc
        int eL, eR;

        noInterrupts();
        eL = encoder1_val;
        eR = encoder2_val;
        interrupts();

        int diff_err = (eL + eR);   // hiệu giữa 2 bánh
        int diff_fix = diff_err * 0.1f;  // hệ số hiệu chỉnh nhỏ

        int L = dir * pwm + diff_fix;
        int R = -dir * pwm - diff_fix;

        motor_run(L, R);

        delay(4);   // chu kỳ mềm, mượt
    }

    // stop cứng
    motor_stop();
    delay(TURN_SETTLE_MS);  // ổn định góc
    reset_encoder();
}

  void turn_left_90()  { turn_90_profiled(-1); }
  void turn_right_90() { turn_90_profiled(+1); }

  // ------------------------------------xoay góc theo hướng tốt nhất-------------------------

void near_wall() {
  float target_dist = 5.0;
  unsigned long start_time = millis();

  while (millis() - start_time < 1500) {
    long sum3 = 0;
    for (int i = 0; i < 5; i++) {
      sum3 += analogRead(IR_3);
    }

    float current_dist = getDistance(sum3 / 5);

    if (current_dist < 0 || current_dist > 17.0) {
      motor_stop();
      return;
    }

    float error = current_dist - target_dist;

    // đã đủ xa hoặc xa hơn target rồi thì thôi
    if (error >= -0.3f) break;

    if (error < -1.0f) motor_run(-80, -75);
    else               motor_run(-55, -50);

    delay(10);
  }

  motor_stop();
  delay(100);
}

void check_wall() {
  int ir1 = analogRead(IR_1);
  int ir5 = analogRead(IR_5);
  int ir3 = analogRead(IR_3);

  if(ir5 < 500) {
    turn_right_90();
  }
  else if(ir1 < 500) {
    turn_left_90();
  }
  else if(ir1 > 1000 && ir5 > 1000 && ir3 > 1000) {
    near_wall();
    turn_right_90();
    near_wall();
    turn_right_90();
  }
}

void rotate(Heading &h, Heading target) {
  int diff = (target - h + 4) % 4;
  DistMM d = read_sensor();
  bool left_wall  = d.l_ok && d.l < 120;
  bool right_wall = d.r_ok && d.r < 120;
  bool front_wall = d.f_ok && d.f < 120;

  if(diff == 1) {
    turn_right_90();
  }      
  else if(diff == 2) {
    // xoay 180 độ
    near_wall(); 
    turn_left_90(); 
    near_wall(); 
    turn_left_90(); 
  }
  else if(diff == 3) {
    turn_left_90();
  }
  h = target;
}

// -------------------------------------------------------
// ============= chạy 1 ô và PID ==============

int click = 0;
float dis_l;
float dis_r;
float d_max = 18;
float d = 0;

float last_l = 0;
float last_r = 0;

// float right = 6;
// float left = 7.5;
float right;
float left;

// void PID() {
//   // 1. TÍNH TOÁN THỜI GIAN THỰC (dt)
//   static unsigned long previousTime = 0; // static giúp giữ giá trị sau mỗi vòng lặp
//   unsigned long currentTime = millis();
  
//   // Tránh lỗi chia cho 0 hoặc sai số ở vòng lặp đầu tiên khi xe mới bật
//   if (previousTime == 0) {
//     previousTime = currentTime;
//     return; 
//   }
  
//   // Tính dt (khoảng thời gian giữa 2 lần tính PID) và đổi ra Giây (seconds)
//   float dt = (currentTime - previousTime) / 1000.0; 
//   previousTime = currentTime;

//   // 2. ĐỌC CẢM BIẾN (Giữ nguyên logic của bạn)
//   long sum2 = 0;
//   long sum4 = 0;
//   long sum5 = 0;
//   long sum1 = analogRead(IR_1);

//   for(int i = 0; i < 10; i++){
//       sum2 += analogRead(IR_2);
//       sum4 += analogRead(IR_4);
//       sum5 += analogRead(IR_5);
//   }

//   int currentAnalog_2 = sum2 / 10;
//   int currentAnalog_4 = sum4 / 10;
//   int currentAnalog_5 = sum5 / 10;
//   float dist_2 = getDistance(currentAnalog_2);
//   float dist_4 = getDistance(currentAnalog_4) - 1;
//   float dist_5 = getDistance(currentAnalog_5);
//   float dist_1 = getDistance(sum1);

//   // long sum1 = 0;
//   // long sum2 = 0;
//   // long sum3 = 0;
//   // long sum4 = 0;
//   // long sum5 = 0;
  
//   // for(int i = 0; i < 20; i++){
//   //     sum1 += analogRead(IR_1);
//   //     sum2 += analogRead(IR_2);
//   //     sum3 += analogRead(IR_3);
//   //     sum4 += analogRead(IR_4);
//   //     sum5 += analogRead(IR_5);
//   //     delay(2);
//   // }
//   // int currentAnalog_1 = sum1 / 20;
//   // int currentAnalog_2 = sum2 / 20;
//   // int currentAnalog_3 = sum3 / 20;
//   // int currentAnalog_4 = sum4 / 20;
//   // int currentAnalog_5 = sum5 / 20;

//   // // 2. Tính toán khoảng cách
//   // float dist_1 = getDistance(currentAnalog_1);
//   // float dist_2 = getDistance(currentAnalog_2);
//   // float dist_3 = getDistance(currentAnalog_3);
//   // float dist_4 = getDistance(currentAnalog_4);
//   // float dist_5 = getDistance(currentAnalog_5);

//   // 3. TÍNH TOÁN PID CHUẨN VỚI THỜI GIAN (dt)

//   if((dist_5 > 10 && dist_1 > 10) || (dist_2 > 15 && dist_4 > 15)) {
//     noInterrupts();
//     int EL = encoder1_val;
//     int ER = encoder2_val;
//     interrupts();

//     e = EL - ER;
//     kp = 0.1;
//     // kp = 0;
//     ki = 0;
//     kd = 0;
//   }
//   else if((dist_2 - left > 0.5 && dist_4 < 10) || dist_1 > 15) {
//     kp = 2;
//     kd = 5;
//     ki = 0;
//     e = 2*(right + 4 - dist_4);

//     // last_r = dist_4;
//     // e = dist_4 - last_r;
//   }
//   else if((dist_4 - right > 0.5 && dist_2 < 10) || dist_5 > 15) {
//     kp = 2;
//     kd = 5;
//     ki = 0;
//     // last_r = dist_2;
//     // e = dist_2 - last_r;
//     e = 2*(dist_2 - left);
//   }
//   else {
//     kp = 15;
//     kd = 0;
//     float e_right = dist_4 - right;
//     float e_left = dist_2 - left;
//     e = e_left - e_right;
//     // last_l = dist_2;
//     // last_r = dist_4;
//   }

//   // Đạo hàm: Tốc độ thay đổi của lỗi chia cho thời gian
//   float dedt = (e - eprev) / dt;    
  
//   // Tích phân: Cộng dồn lỗi nhân với thời gian
//   eintegral += (e * dt);            

//   float u = kp * e + kd * dedt + ki * eintegral;
//   float correction = constrain(u, -50, 50); 

//   // 4. ĐIỀU KHIỂN ĐỘNG CƠ
//   speed_l = constrain(SPEED_L - correction, -250, 250);
//   speed_r = constrain(SPEED_R + correction, -250, 250);
//   // motor_run(speed_l, speed_r);

//   // 6. CẬP NHẬT TRẠNG THÁI
//   eprev = e;
  
// }

float refL1 = 0, refL2 = 0, refR4 = 0, refR5 = 0;

float filt_d1 = 0, filt_d2 = 0, filt_d4 = 0, filt_d5 = 0;

float confL = 0.0f;
float confR = 0.0f;

const float SENSOR_ALPHA = 0.35f;   // lọc EMA
const float CONF_UP      = 0.18f;   // thấy tường thì tăng nhanh
const float CONF_DOWN    = 0.10f;   // mất tường thì giảm chậm hơn một chút

const float SIDE_MIN = 0.8f;
const float SIDE_MAX = 14.0f;

const float DIAG_MIN = 2.0f;
const float DIAG_MAX = 15.0f;

const float KP_TRACK = 10.0f;
const float KD_TRACK = 0.0f;   // ban đầu để 0, tune sau
const float KI_TRACK = 0.0f;

const float KP_ENC_SYNC = 0.08f; // giữ 2 bánh đồng đều

bool refsReady = false;

void calibrateWallRefs() {
  Serial.println("Start calibrate wall refs...");

  float last1 = 0, last2 = 0, last4 = 0, last5 = 0;
  int stable_count = 0;

  while (stable_count < 10) {
    long sum1 = 0, sum2 = 0, sum4 = 0, sum5 = 0;

    for (int i = 0; i < 20; i++) {
      sum1 += analogRead(IR_1);
      sum2 += analogRead(IR_2);
      sum4 += analogRead(IR_4);
      sum5 += analogRead(IR_5);
      delay(2);
    }

    float d1 = getDistance(sum1 / 20);
    float d2 = getDistance(sum2 / 20);
    float d4 = getDistance(sum4 / 20);
    float d5 = getDistance(sum5 / 20);

    Serial.print("d1="); Serial.print(d1);
    Serial.print(" d2="); Serial.print(d2);
    Serial.print(" d4="); Serial.print(d4);
    Serial.print(" d5="); Serial.println(d5);

    if (fabs(d1 - last1) < 0.15f &&
        fabs(d2 - last2) < 0.15f &&
        fabs(d4 - last4) < 0.15f &&
        fabs(d5 - last5) < 0.15f) {
      stable_count++;
    } else {
      stable_count = 0;
    }

    last1 = d1;
    last2 = d2;
    last4 = d4;
    last5 = d5;

    delay(40);
  }

  refL1 = last1;
  refL2 = last2;
  refR4 = last4;
  refR5 = last5;

  filt_d1 = refL1;
  filt_d2 = refL2;
  filt_d4 = refR4;
  filt_d5 = refR5;

  confL = 1.0f;
  confR = 1.0f;

  Serial.println("Calibration done!");
  Serial.print("refL1 = "); Serial.println(refL1);
  Serial.print("refL2 = "); Serial.println(refL2);
  Serial.print("refR4 = "); Serial.println(refR4);
  Serial.print("refR5 = "); Serial.println(refR5);

  refsReady = true;
}

bool validSide(float d) {
  return (d > SIDE_MIN && d < SIDE_MAX);
}

bool validDiag(float d) {
  return (d > DIAG_MIN && d < DIAG_MAX);
}

void readWallSensors(float &d1, float &d2, float &d4, float &d5) {
  long sum1 = 0, sum2 = 0, sum4 = 0, sum5 = 0;

  for (int i = 0; i < 8; i++) {
    sum1 += analogRead(IR_1);
    sum2 += analogRead(IR_2);
    sum4 += analogRead(IR_4);
    sum5 += analogRead(IR_5);
  }

  float raw1 = getDistance(sum1 / 8);
  float raw2 = getDistance(sum2 / 8);
  float raw4 = getDistance(sum4 / 8);
  float raw5 = getDistance(sum5 / 8);

  if (filt_d1 == 0) {
    filt_d1 = raw1;
    filt_d2 = raw2;
    filt_d4 = raw4;
    filt_d5 = raw5;
  } else {
    filt_d1 = SENSOR_ALPHA * raw1 + (1.0f - SENSOR_ALPHA) * filt_d1;
    filt_d2 = SENSOR_ALPHA * raw2 + (1.0f - SENSOR_ALPHA) * filt_d2;
    filt_d4 = SENSOR_ALPHA * raw4 + (1.0f - SENSOR_ALPHA) * filt_d4;
    filt_d5 = SENSOR_ALPHA * raw5 + (1.0f - SENSOR_ALPHA) * filt_d5;
  }

  d1 = filt_d1;
  d2 = filt_d2;
  d4 = filt_d4;
  d5 = filt_d5;
}

void updateWallConfidence(float d1, float d5) {
  bool seeLeftWall  = validSide(d1);
  bool seeRightWall = validSide(d5);

  if (seeLeftWall) confL += CONF_UP;
  else             confL -= CONF_DOWN;

  if (seeRightWall) confR += CONF_UP;
  else              confR -= CONF_DOWN;

  confL = constrain(confL, 0.0f, 1.0f);
  confR = constrain(confR, 0.0f, 1.0f);
}

float computeWallError(float d1, float d2, float d4, float d5) {
  float eL = 0.0f, eR = 0.0f;
  float nL = 0.0f, nR = 0.0f;

  // BÊN TRÁI
  if (validSide(d1)) {
    eL += 0.75f * (d1 - refL1);   // cạnh trái là chính
    nL += 0.75f;
  }
  if (validDiag(d2)) {
    eL += 0.25f * (d2 - refL2);   // chéo trái là phụ
    nL += 0.25f;
  }

  // BÊN PHẢI
  if (validSide(d5)) {
    eR += 0.75f * (refR5 - d5);   // đổi dấu để cùng quy ước với bên trái
    nR += 0.75f;
  }
  if (validDiag(d4)) {
    eR += 0.25f * (refR4 - d4);
    nR += 0.25f;
  }

  if (nL > 0.0f) eL /= nL;
  if (nR > 0.0f) eR /= nR;

  float wL = (nL > 0.0f) ? confL : 0.0f;
  float wR = (nR > 0.0f) ? confR : 0.0f;

  if ((wL + wR) > 0.001f) {
    return (wL * eL + wR * eR) / (wL + wR);
  }

  return 0.0f;
}

// void PID() {
//   static unsigned long previousTime = 0;
//   unsigned long currentTime = millis();

//   if (previousTime == 0) {
//     previousTime = currentTime;
//     return;
//   }

//   float dt = (currentTime - previousTime) / 1000.0f;
//   previousTime = currentTime;

//   float d1, d2, d4, d5;
//   readWallSensors(d1, d2, d4, d5);

//   updateWallConfidence(d1, d5);

//   float e_wall = computeWallError(d1, d2, d4, d5);

//   noInterrupts();
//   int EL = encoder1_val;
//   int ER = encoder2_val;
//   interrupts();

//   float e_enc = KP_ENC_SYNC * (EL - ER);

//   bool leftSeen  = confL > 0.6f;
//   bool rightSeen = confR > 0.6f;

//   float wallGain = 0.0f;

//   if (leftSeen && rightSeen) {
//     wallGain = 1.0f;     // 2 tường -> bám giữa mạnh
//   } 
//   else if (leftSeen || rightSeen) {
//     wallGain = 0.35f;    // 1 tường -> chỉ dùng nhẹ thôi
//   } 
//   else {
//     wallGain = 0.0f;     // không tường -> chỉ giữ hướng
//   }

//   e = wallGain * e_wall + e_enc;

//   float dedt = (e - eprev) / dt;
//   eintegral += e * dt;

//   float u = KP_TRACK * e + KD_TRACK * dedt + KI_TRACK * eintegral;
//   float correction = constrain(u, -40, 40);

//   speed_l = constrain(SPEED_L - correction, -250, 250);
//   speed_r = constrain(SPEED_R + correction, -250, 250);

//   eprev = e;
// }

// void PID() {
//   static unsigned long previousTime = 0;
//   unsigned long currentTime = millis();

//   if (previousTime == 0) {
//     previousTime = currentTime;
//     return;
//   }

//   float dt = (currentTime - previousTime) / 1000.0f;
//   previousTime = currentTime;

//   float d1, d2, d4, d5;
//   readWallSensors(d1, d2, d4, d5);

//   updateWallConfidence(d1, d5);

//   float e_wall = computeWallError(d1, d2, d4, d5);

//   noInterrupts();
//   int EL = encoder1_val;
//   int ER = encoder2_val;
//   interrupts();

//   float e_enc = KP_ENC_SYNC * (EL - ER);

//   // luôn giữ chút đồng tốc encoder
//   e = e_wall + e_enc;

//   float dedt = (e - eprev) / dt;
//   eintegral += e * dt;

//   float u = KP_TRACK * e + KD_TRACK * dedt + KI_TRACK * eintegral;
//   float correction = constrain(u, -50, 50);

//   speed_l = constrain(SPEED_L - correction, -250, 250);
//   speed_r = constrain(SPEED_R + correction, -250, 250);

//   eprev = e;
// }

bool intersectionMode = false;
float intersectionStartDist = 0.0f;

float prev_d1 = 0.0f;
float prev_d5 = 0.0f;
bool sideHistoryReady = false;

const float OPEN_JUMP_CM = 1.2f;          // cảm biến cạnh tăng đột ngột => gặp cửa/ngã tư
const float INTERSECTION_HOLD_CM = 1.5f;  // đoạn chỉ giữ hướng bằng encoder
const float INTERSECTION_BLEND_CM = 1.0f; // đoạn bật lại PID từ từ

int encBiasAtIntersection = 0;

void PID() {
  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();

  if (!refsReady) {
    speed_l = SPEED_L;
    speed_r = SPEED_R;
    return;
  }

  if (previousTime == 0) {
    previousTime = currentTime;
    return;
  }

  float dt = (currentTime - previousTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  previousTime = currentTime;

  float d1, d2, d4, d5;
  readWallSensors(d1, d2, d4, d5);
  updateWallConfidence(d1, d5);

  if (!sideHistoryReady) {
  prev_d1 = d1;
  prev_d5 = d5;
  sideHistoryReady = true;
}

// bool leftOpenedSuddenly  = validSide(prev_d1) && (d1 - prev_d1 > OPEN_JUMP_CM);
// bool rightOpenedSuddenly = validSide(prev_d5) && (d5 - prev_d5 > OPEN_JUMP_CM);

// // chỉ xét khi robot đã đi vào giữa cell một chút, tránh false trigger ngay sau lúc mới xoay
// bool crossingHint = (leftOpenedSuddenly || rightOpenedSuddenly) && (d > 3.0f) && (d < d_max - 3.0f);

// if (!intersectionMode && crossingHint) {
//   intersectionMode = true;
//   intersectionStartDist = d;
//   eprev = 0;
//   eintegral = 0;
//   // Serial.println("INTERSECTION DETECTED");
// }

bool leftOpenedSuddenly  = validSide(prev_d1) && (d1 - prev_d1 > OPEN_JUMP_CM);
bool rightOpenedSuddenly = validSide(prev_d5) && (d5 - prev_d5 > OPEN_JUMP_CM);

bool leftDiagOpened  = validDiag(d2) && ((d2 - refL2) > 1.0f);
bool rightDiagOpened = validDiag(d4) && ((d4 - refR4) > 1.0f);

bool crossingHint =
    (leftOpenedSuddenly || rightOpenedSuddenly || leftDiagOpened || rightDiagOpened) &&
    (d > 3.0f) && (d < d_max - 3.0f);

if (!intersectionMode && crossingHint) {
  intersectionMode = true;
  intersectionStartDist = d;
  // eprev = 0;
  eintegral = 0;

  noInterrupts();
  encBiasAtIntersection = encoder1_val - encoder2_val;
  interrupts();
}

  bool leftSeen  = confL > 0.6f;
  bool rightSeen = confR > 0.6f;

  float e_wall = 0.0f;
  float kp_now = 0.0f;
  float kd_now = 0.0f;
  float ki_now = 0.0f;

  if (leftSeen && rightSeen) {
    // ===== 2 TƯỜNG: giữ giữa hành lang =====
    float e_side = (d1 - refL1) - (d5 - refR5);
    float e_diag = (d2 - refL2) - (d4 - refR4);

    e_wall = 0.85f * e_side + 0.15f * e_diag;

    kp_now = 10.0f;
    kd_now = 2.0f;
    ki_now = 0.0f;
  }
  else if (leftSeen) {
    float e_side = (d1 - refL1);
    float e_diag = (d2 - refL2);

    // --- FIX: Chống bám mép tường (Corner Pull) ---
    // Cho phép xe lách ra xa tường (âm), nhưng giới hạn việc xe bẻ lái đâm vào khoảng trống (dương)
    e_side = constrain(e_side, -1.0f, 0.5f); 
    e_diag = constrain(e_diag, -1.0f, 0.5f);

    // Xóa bỏ + 0.2f trừ khi phần cứng của bạn bắt buộc phải có để bù lệch cơ khí
    e_wall = 0.85f * e_side + 0.15f * e_diag;

    // nếu đang chéo đầu vào tường trái thì phản ứng mạnh (âm)
    if (e_diag < -0.8f || e_side < -0.8f) {
        kp_now = 26.0f;
        kd_now = 6.0f;
    } else {
        kp_now = 5.0f;
        kd_now = 2.5f;
    }
    ki_now = 0.0f;
  }
  else if (rightSeen) {
    float e_side = -(d5 - refR5);
    float e_diag = -(d4 - refR4);

    // --- FIX: Chống bám mép tường (Corner Pull) ---
    e_side = constrain(e_side, -1.0f, 0.5f);
    e_diag = constrain(e_diag, -1.0f, 0.5f);

    e_wall = 0.85f * e_side + 0.15f * e_diag;

    if (e_diag > 0.8f || e_side > 0.8f) {
        kp_now = 26.0f;
        kd_now = 6.0f;
    } else {
        kp_now = 5.0f;
        kd_now = 2.5f;
    }
    ki_now = 0.0f;
  }
  else {
    // ===== KHÔNG TƯỜNG: giữ thẳng bằng encoder =====
    e_wall = 0.0f;
    kp_now = 0.0f;
    kd_now = 0.0f;
    ki_now = 0.0f;
  }

  if (intersectionMode) {
  float passed = d - intersectionStartDist;

  if (passed < INTERSECTION_HOLD_CM) {
    // đi xuyên qua ngã tư: bỏ wall PID, chỉ giữ hướng encoder
    e_wall = 0.0f;
    kp_now = 5.0f;
    kd_now = 2.0f;
    ki_now = 0.0f;
  }
  else if (passed < INTERSECTION_HOLD_CM + INTERSECTION_BLEND_CM) {
    // bật lại từ từ để khỏi giật
    float alpha = (passed - INTERSECTION_HOLD_CM) / INTERSECTION_BLEND_CM;
    e_wall *= alpha;
    // kp_now *= alpha;
    // kd_now *= alpha;
    // ki_now *= alpha;
  }
  else {
    intersectionMode = false;
    // eprev = 0;
    eintegral = 0;
    Serial.println("INTERSECTION END");
  }
}

  noInterrupts();
int EL = encoder1_val;
int ER = encoder2_val;
interrupts();

float e_enc;
if (intersectionMode) {
  e_enc = 0.02f * ((EL - ER) - encBiasAtIntersection);
} else {
  e_enc = 0.01f * (EL - ER);
}

  e = e_wall + e_enc;

  float dedt = (e - eprev) / dt;
  eintegral += e * dt;

  float u = kp_now * e + kd_now * dedt + ki_now * eintegral;
  float correction = constrain(u, -80, 80);

  speed_l = constrain(SPEED_L - correction, -250, 250);
  speed_r = constrain(SPEED_R + correction, -250, 250);

  eprev = e;
  prev_d1 = d1;
  prev_d5 = d5;

  Serial.print("d1="); Serial.print(d1);
  Serial.print(" d5="); Serial.print(d5);
  Serial.print(" e_wall="); Serial.print(e_wall);
  Serial.print(" corr="); Serial.println(correction);
}

#define ban_kinh    1.05f     // bán kính bánh xe
#define xung        406.0f   // số xung 1 vòng  
#define pi          3.14159f // số pi 

float distance_left() {
  float chu_vi = 2 * ban_kinh * pi;

  noInterrupts();
  float e1 = encoder1_val;
  interrupts();

  float vong = e1 / xung;

  return vong * chu_vi;
}

float distance_right() {
  float chu_vi = 2 * ban_kinh * pi;

  noInterrupts();
  float e2 = encoder2_val;
  interrupts();

  float vong = e2 / xung;

  return vong * chu_vi;
}

// ============= CẤU HÌNH TỐC ĐỘ NÂNG CAO =================
#define MAX_SPEED    100   // Tốc độ tối đa
#define START_PWM    70    // Tốc độ khởi động (cao để thắng ma sát nghỉ)
#define CRAWL_PWM    45    // Tốc độ "bò" về đích (thấp để chính xác)
#define CRAWL_DIST   2.0   // Khoảng cách bắt đầu bò (cách đích 5cm)

int cal = 0; // Biến trạng thái: 0=chưa chạy, 1=đang chạy, 3=đã xong

// Thêm biến global ở đầu chương trình hoặc trong loop
unsigned long last_time_run = 0;
const int TIME_INTERVAL = 20; // 20ms (tương đương 50 lần/giây)

bool motor_run_fast() {
  // Lưu trạng thái ban đầu
  unsigned long startTime = millis();
  bool moving = true;

  // Vòng lặp while để di chuyển cho đến khi đủ 1 ô
  while (moving) {
    if(millis() - last_time_run >= TIME_INTERVAL) {
      last_time_run = millis();
    float d_l = distance_left();
    float d_r = distance_right();

    d = (fabs(d_l) + fabs(d_r)) / 2;
    // Kiểm tra nếu robot đã di chuyển đủ khoảng cách của 1 ô
    if (d >= d_max) {
  motor_stop();
  delay(50);
  reset_encoder();
  near_wall();
  d = 0;
  reset_encoder();
  return true;
}

    // Nếu chưa đạt đích, tiếp tục di chuyển
    else {
      // Cấu hình tốc độ dựa trên giai đoạn
      float target_speed = 0;
      float mid_point = d_max / 2.0;  // Giới hạn giữa (ví dụ 20cm)
      float crawl_point = d_max - CRAWL_DIST;  // Điểm bắt đầu giảm tốc (35cm)

      // --- GIAI ĐOẠN 1: TĂNG TỐC (0 -> 20cm) ---
      if (d < mid_point) {
        target_speed = START_PWM + (d / mid_point) * (MAX_SPEED - START_PWM);
      } 

      // --- GIAI ĐOẠN 2: GIẢM TỐC (20cm -> 35cm) ---
      else if (d < crawl_point) {
        float distance_in_decel = d - mid_point;
        float decel_range = crawl_point - mid_point;
        target_speed = MAX_SPEED - (distance_in_decel / decel_range) * (MAX_SPEED - CRAWL_PWM);
      }

      // --- GIAI ĐOẠN 3: BÒ VỀ ĐÍCH (35cm -> 40cm) ---
      else {
        target_speed = CRAWL_PWM;
      }

      // Gán tốc độ và điều khiển động cơ
      SPEED_L = constrain(target_speed, CRAWL_PWM, MAX_SPEED);
      SPEED_R = SPEED_L * 0.98;  // Điều chỉnh cân bằng tốc độ bánh trái và phải

      PID();  // Điều khiển bằng PID

      motor_run(speed_l, speed_r);  // Điều khiển động cơ

      // Debug: In ra quá trình "hạ cánh"
      Serial.print("D: "); Serial.println(d);
    }
    }

    // Kiểm tra thời gian để tránh vòng lặp vô tận (đảm bảo robot không bị kẹt)
    if (millis() - startTime > 10000) {
  motor_stop();
  Serial.println("Timeout reached. Stopping.");
  return false;
}
  }
  return false;
}
void reset_PID_vars() {
  eprev = 0;
  eintegral = 0;
  // Reset các biến control khác nếu cần
}

void finalizeUnknownWalls() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      if (maze[i][j] == 4) maze[i][j] = 2;   // ?   -> |
      else if (maze[i][j] == 5) maze[i][j] = 3; // ??? -> ---
    }
  }
}

bool saveMazeToFlash() {
  prefs.begin("micromouse", false);

  prefs.putBool("valid", true);
  prefs.putULong("ver", MAP_VERSION);
  prefs.putInt("goalx", goal_x2);
  prefs.putInt("goaly", goal_y2);
  prefs.putInt("msize", MAZE_SIZE);

  size_t written = prefs.putBytes("maze", maze, sizeof(maze));

  prefs.end();

  return (written == sizeof(maze));
}

bool loadMazeFromFlash() {
  prefs.begin("micromouse", true);

  bool valid = prefs.getBool("valid", false);
  uint32_t ver = prefs.getULong("ver", 0);
  int gx = prefs.getInt("goalx", -1);
  int gy = prefs.getInt("goaly", -1);
  int msize = prefs.getInt("msize", -1);

  if (!valid) {
    prefs.end();
    Serial.println("FLASH: no valid map");
    return false;
  }

  if (ver != MAP_VERSION) {
    prefs.end();
    Serial.println("FLASH: version mismatch");
    return false;
  }

  if (gx != goal_x2 || gy != goal_y2) {
    prefs.end();
    Serial.println("FLASH: goal mismatch");
    return false;
  }

  if (msize != MAZE_SIZE) {
    prefs.end();
    Serial.println("FLASH: maze size mismatch");
    return false;
  }

  size_t readBytes = prefs.getBytes("maze", maze, sizeof(maze));
  prefs.end();

  if (readBytes != sizeof(maze)) {
    Serial.println("FLASH: maze bytes mismatch");
    return false;
  }

  Serial.println("FLASH: map loaded OK");
  return true;
}

void clearMazeFlash() {
  prefs.begin("micromouse", false);
  prefs.clear();
  prefs.end();
  Serial.println("FLASH: cleared");
}

void appendPathCmd(char c) {
  if (plannedLen < (int)sizeof(plannedPath) - 1) {
    plannedPath[plannedLen++] = c;
    plannedPath[plannedLen] = '\0';
  }
}

Heading chooseBestDirectionSolved(int mx, int my, Heading h) {
  int best = 9999;
  Heading bestDir = h;

  for (int ddir = 0; ddir < 4; ddir++) {
    int nx = mx + dx[ddir];
    int ny = my + dy[ddir];

    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;
    if (!canMove(mx, my, nx, ny)) continue;
    if (dist[nx][ny] < 0) continue;

    if (dist[nx][ny] < best) {
      best = dist[nx][ny];
      bestDir = (Heading)ddir;
    }
    else if (dist[nx][ny] == best && (Heading)ddir == h) {
      bestDir = (Heading)ddir;   // ưu tiên đi thẳng nếu bằng nhau
    }
  }

  return bestDir;
}

bool buildPlannedPath(int startMx, int startMy, Heading startH,
                      int goalMx, int goalMy) {
  plannedLen = 0;
  plannedPath[0] = '\0';

  floodFill(goalMx, goalMy);

  if (dist[startMx][startMy] < 0) {
    return false;
  }

  int mx = startMx;
  int my = startMy;
  Heading h = startH;

  int guard = 0;
  while (!(mx == goalMx && my == goalMy) && guard < 200) {
    Heading nextH = chooseBestDirectionSolved(mx, my, h);

    int nx = mx + dx[nextH];
    int ny = my + dy[nextH];

    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) return false;
    if (!canMove(mx, my, nx, ny)) return false;
    if (dist[nx][ny] < 0) return false;

    int diff = (nextH - h + 4) % 4;

    if (diff == 1) {
      appendPathCmd('R');
    }
    else if (diff == 3) {
      appendPathCmd('L');
    }
    else if (diff == 2) {
      appendPathCmd('R');
      appendPathCmd('R');
    }

    appendPathCmd('F');

    mx = nx;
    my = ny;
    h = nextH;
    guard++;
  }

  return (mx == goalMx && my == goalMy);
}

void printPlannedPathCompact() {
  Serial.print("PATH RAW: ");
  Serial.println(plannedPath);

  Serial.print("PATH COMPACT: ");
  int i = 0;
  while (i < plannedLen) {
    if (plannedPath[i] == 'F') {
      int cnt = 0;
      while (i < plannedLen && plannedPath[i] == 'F') {
        cnt++;
        i++;
      }
      Serial.print(cnt);
      Serial.print("F ");
    } else {
      Serial.print(plannedPath[i]);
      Serial.print(" ");
      i++;
    }
  }
  Serial.println();
}

void motor_run_distance_cm(float target_cm, int pwm_cmd = 100) {
  d_max = target_cm;

  reset_encoder();
  reset_PID_vars();

  intersectionMode = false;
  sideHistoryReady = false;
  prev_d1 = 0.0f;
  prev_d5 = 0.0f;
  d = 0.0f;

  unsigned long startTime = millis();

  while (1) {
    if (millis() - last_time_run >= TIME_INTERVAL) {
      last_time_run = millis();

      float d_l = distance_left();
      float d_r = distance_right();
      d = (fabs(d_l) + fabs(d_r)) / 2.0f;

      if (d >= target_cm) {
        motor_stop();
        delay(50);
        reset_encoder();
        d = 0.0f;
        return;
      }

      float remain = target_cm - d;
      float target_speed = pwm_cmd;

      // giảm tốc nhẹ khi gần hết đoạn
      if (remain < 4.0f) {
        target_speed = CRAWL_PWM + (remain / 4.0f) * (pwm_cmd - CRAWL_PWM);
        if (target_speed < CRAWL_PWM) target_speed = CRAWL_PWM;
      }

      SPEED_L = constrain(target_speed, (float)CRAWL_PWM, 250.0f);
      SPEED_R = SPEED_L * 0.98f;

      PID();
      motor_run(speed_l, speed_r);
    }

    if (millis() - startTime > 15000) {
      motor_stop();
      Serial.println("run_distance timeout");
      return;
    }
  }
}

void executePlannedPath() {
  int i = 0;

  while (i < plannedLen) {
    while (i < plannedLen && (plannedPath[i] == 'L' || plannedPath[i] == 'R')) {
      if (plannedPath[i] == 'L') {
        turn_left_90();
      } else {
        turn_right_90();
      }
      i++;
    }

    int fCount = 0;
    while (i < plannedLen && plannedPath[i] == 'F') {
      fCount++;
      i++;
    }

    if (fCount > 0) {
      float segment_cm = CELL_LEN_CM * fCount;

      Serial.print("RUN ");
      Serial.print(fCount);
      Serial.print(" cells = ");
      Serial.print(segment_cm);
      Serial.println(" cm");

      motor_run_distance_cm(segment_cm, 100);   // 100 hiện tại là PWM
    }
  }

  motor_stop();
  Serial.println("FAST RUN DONE");
}

void resetMazeToDefault() {
  memcpy(maze, maze_default, sizeof(maze_default));
  memset(dist, -1, sizeof(dist));

  cx = 1;
  cy = 1;
  heading = EAST;

  plannedLen = 0;
  plannedPath[0] = '\0';

  savedMapAvailable = false;
  fastRunReady = false;

  reset_encoder();
  reset_PID_vars();

  intersectionMode = false;
  sideHistoryReady = false;
  prev_d1 = 0.0f;
  prev_d5 = 0.0f;
  d = 0.0f;

  Serial.println("Maze reset to default.");
}

void setup() {
  Serial.begin(115200);

  pinMode(C1_LEFT, INPUT);
  pinMode(C1_RIGHT, INPUT);
  pinMode(C2_LEFT, INPUT);
  pinMode(C2_RIGHT, INPUT);

  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(button_1, INPUT_PULLUP);
  pinMode(button_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(C1_LEFT), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C1_RIGHT), encoder2, CHANGE);

  SerialBT.begin("ESP32_ROBOT"); // Tên Bluetooth hiển thị

  Serial.println("Bluetooth started");

  delay(200);

  // Giữ nút 2 lúc khởi động để xóa flash
  if (digitalRead(button_2) == LOW) {
    Serial.println("BUTTON_2 held -> clearing flash...");
    clearMazeFlash();
    delay(1000);
  }

  // Chỉ load nếu cho phép
  if (enableLoadFromFlash) {
    if (loadMazeFromFlash()) {
      savedMapAvailable = true;
      finalizeUnknownWalls();

      Serial.println("Loaded maze from flash.");

      if (buildPlannedPath(goal_x1, goal_y1, FAST_START_HEADING, goal_x2, goal_y2)) {
        fastRunReady = true;
        printPlannedPathCompact();
      } else {
        Serial.println("Saved map exists, but path build failed.");
      }
    } else {
      Serial.println("No usable maze in flash.");
    }
  } else {
    Serial.println("Load from flash is disabled.");
  }
}

void loop() {
  // button_1: explore
  if (digitalRead(button_1) == LOW && click == 0) {
    delay(300);
    resetMazeToDefault();
    calibrateWallRefs();
    reset_PID_vars();
    reset_encoder();
    click = 1;
  }

  // button_2: fast run bằng map đã lưu
  else if (digitalRead(button_2) == LOW && click == 0 && fastRunReady) {
    delay(300);
    calibrateWallRefs();
    reset_PID_vars();
    reset_encoder();
    click = 3;
  }

  // ===== CHẾ ĐỘ KHÁM PHÁ =====
  else if (click == 1) {
    detect_update_maze(cx, cy, heading);

    floodFill(goal_x2, goal_y2);

    Heading target = chooseBestDirection(cx, cy, heading);

    rotate(heading, target);
    bool moved_ok = motor_run_fast();

if (moved_ok) {
  if (heading == NORTH) cx--;
  else if (heading == SOUTH) cx++;
  else if (heading == EAST)  cy++;
  else if (heading == WEST)  cy--;
}

    printMazeBT();

    if (cx * 2 - 1 == goal_x2 && cy * 2 - 1 == goal_y2) {
      motor_stop();

      // 1) khóa toàn bộ unknown thành tường
      finalizeUnknownWalls();

      Serial.println("Reached goal. Finalized unknown walls.");
      printMazeBT();

      // 2) lưu flash
      if (saveMazeToFlash()) {
        Serial.println("Maze saved to flash.");
      } else {
        Serial.println("Maze save FAILED.");
      }

      // 3) build đường đi nhanh từ goal_1 -> goal_2
      if (buildPlannedPath(goal_x1, goal_y1, FAST_START_HEADING, goal_x2, goal_y2)) {
        fastRunReady = true;
        printPlannedPathCompact();
      } else {
        Serial.println("Cannot build fast path.");
      }

      click = 2;
    }
  }

  // ===== CHẾ ĐỘ CHẠY NHANH =====
  else if (click == 3) {
    executePlannedPath();
    click = 2;
  }
}
