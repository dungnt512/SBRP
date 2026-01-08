#include <bits/stdc++.h>
using namespace std;

using ll = long long;
using db = double;
using ld = long double;

mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

ll rnd(ll l, ll r) {
  return uniform_int_distribution<ll>(l, r)(rng);
}
ld rndf(ld l, ld r) {
  return uniform_real_distribution<ld>(l, r)(rng);
}

string to_str(ll x) {
  if (!x) return "0";
  string res;
  while (x) {
    res += x % 10 ^ '0';
    x /= 10;
  }
  reverse(res.begin(), res.end());
  return res;
}

int tmp_i;
db tmp_db;
char tmp_c;
char tmp_s[256];

string inputs[] = {
  "Canberra",
  "Adelaide",
  "Bridgend",
  "Brisbane",
  "Cardiff",
  "Edinburgh-1",
  "Edinburgh-2",
  "MiltonKeynes",
  "Porthcawl",
  "Suffolk"
};

struct Stop {
  db latitude, longitude;
  char name[256];
};
struct Addr {
  db latitude, longitude;
  int num_passengers;
  char name[256];
};
struct Drive {
  db time = INFINITY, distance = INFINITY;
};
struct Walk {
  int stop;
  db time = INFINITY, distance = INFINITY;
};

int num_stops, num_addr, num_walks;
db m_e, m_w;
char unit;

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args ) {
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

int main() {
  cin.tie(0)->sync_with_stdio(0);

  int iter = 0;
  for (auto &input : inputs) {
    FILE *fptr = fopen((input + ".bus").c_str(), "r");
    // cerr << fptr << "\n";
    if (!fptr) {
      fptr = fopen((input).c_str(), "r");
      if (!fptr) continue;
    }

    fscanf(fptr, "%d,%d,%d,%c,%lf,%lf,%256[^\n]\n", &num_stops, &num_addr, &num_walks, &unit, &m_e, &m_w, tmp_s);
    // fscanf(fptr, "%c", &tmp_c);
    printf("Input %d: %s\n", iter, input.c_str());
    vector<Stop> stops(num_stops);
    for (int i = 0; i < num_stops; i++) {
      auto &[lat, lon, name] = stops[i];
      fscanf(fptr, "s,%lf,%lf,%256[^\n]\n", &lat, &lon, name);
      // printf("%lf %lf %s\n", lat, lon, name);
    }
    vector<Addr> addrs(num_addr);
    for (int i = 0; i < num_addr; i++) {
      auto &[lat, lon, num_passengers, name] = addrs[i];
      fscanf(fptr, "a,%lf,%lf,%d,%256[^\n]\n", &lat, &lon, &num_passengers, name);
      // printf("%lf %lf %d %s\n", lat, lon, num_passengers, name);
    }

    vector<vector<Drive>> drive(num_stops, vector<Drive>(num_stops));
    for (int i = 0; i < num_stops * num_stops; i++) {
      int from, to;
      fscanf(fptr, "d,%d,%d,", &from, &to);
      auto &[time, distance] = drive[from][to];
      fscanf(fptr, "%lf,%lf\n", &distance, &time);
      // printf("%d %d %.6lf %.6lf\n", from, to, time, distance);
    }

    vector<vector<Walk>> walk(num_addr);
    for (int i = 0; i < num_walks; i++) {
      int from;
      fscanf(fptr, "w,%d,", &from);
      auto &[stop, time, distance] = walk[from].emplace_back();
      fscanf(fptr, "%d,%lf,%lf\n", &stop, &distance, &time);
      // if (distance > m_w) {
      //   printf("Address %d has a walk distance of %.2lf %c to stop %d, exceeding m_w = %.2lf %c\n",
      //           from, distance, unit, stop, m_w, unit);
      // }
    }

    fclose(fptr);

    int m_t = rnd(45, 45);
    int capacity = rnd(70, 70);
    db sec_per_passenger = rndf(5, 5);
    db ser_per_stop = rndf(15, 15);

    string command = string_format(".\\solver -i %s -m %d -c %d -d %lf %lf -S", input.c_str(), m_t, capacity, sec_per_passenger, ser_per_stop);
    printf("%d %d %d %c %.6lf %.6lf %d %d %.6lf %.6lf %s\n", num_stops, num_addr, num_walks, unit, m_e, m_w, m_t, capacity, sec_per_passenger, ser_per_stop, command.c_str());

    FILE *ofptr = fopen((to_str(iter) + ".in").c_str(), "w");
    fprintf(ofptr, "%d %d %d %.6lf %.6lf %d %d %.6lf %.6lf\n", num_stops, num_addr, num_walks, m_e, m_w, m_t, capacity, sec_per_passenger, ser_per_stop);
    for (int i = 0; i < num_stops; i++) {
      fprintf(ofptr, "%.6lf %.6lf\n", stops[i].latitude, stops[i].longitude);
    }
    for (int i = 0; i < num_addr; i++) {
      fprintf(ofptr, "%.6lf %.6lf %d\n", addrs[i].latitude, addrs[i].longitude, addrs[i].num_passengers);
    }
    fprintf(ofptr, "\n");
    for (int i = 0; i < num_stops; i++) {
      for (int j = 0; j < num_stops; j++) {
        fprintf(ofptr, "%.6lf ", drive[i][j].distance);
      }
      fprintf(ofptr, "\n");
    }
    for (int i = 0; i < num_stops; i++) {
      for (int j = 0; j < num_stops; j++) {
        fprintf(ofptr, "%.6lf ", drive[i][j].time);
      }
      fprintf(ofptr, "\n");
    }
    for (int i = 0; i < num_addr; i++) {
      fprintf(ofptr, "%d ", walk[i].size());
      for (auto &w : walk[i]) {
        fprintf(ofptr, "%d %.6lf %.6lf ", w.stop, w.distance, w.time);
      }
      fprintf(ofptr, "\n");
    }
    fclose(ofptr);

    // ll startTime = chrono::high_resolution_clock::now().time_since_epoch().count();
    // // system(("./solver -i " + input).c_str());
    // system(command.c_str());
    // ll endTime = chrono::high_resolution_clock::now().time_since_epoch().count();
    // printf("Solver time: %.3lf seconds\n", (endTime - startTime) / 1e9);

    // // fptr = fopen("result.out", "r");
    // // if (!fptr) {
    // //   printf("No output produced.\n");
    // //   fclose(fptr);
    // //   continue;
    // // }

    // // ofptr = fopen((to_str(iter) + ".out").c_str(), "w");
    // // while (fscanf(fptr, "%s", tmp_s) != EOF) {
    // //   fprintf(ofptr, "%s ", tmp_s);
    // // }
    // ifstream result("result.out");
    // ofstream out(to_str(iter) + ".out");
    // string line;
    // while (getline(result, line)) {
    //   out << line << "\n";
    // }

    iter++;
  }
  
  return 0;
}