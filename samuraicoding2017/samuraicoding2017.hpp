#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <memory>
#include <map>
#include <chrono>
#include <random>

using namespace std;


#define USE_TT
#define TT_COUNT

#define MYOBED

#define MYAB
#define MYAB2

#define MYHYOUKA

#define USE_ALPHABETA
#define CALCNB

#define CHANGEDEPTH

//#define RECORDHIST
//#define DEBUG_DISPHYOUKA

#ifdef RECORDHIST
#undef USE_ALPHABETA
#undef CHANGEDEPTH
#endif

//#define DEBUG
#define DEBUGCOUNT
enum {
	D_AB1,
	D_AB2,
	D_IS1,
	D_IS2,
	D_TT1,
	D_NTT1,
	D_OB1,
	D_OB2,
	D_OB3,
	D_OB4,
	D_O1,
	D_O2,
	DNUM
};

#define MAXWIDTH 20
#define MAXHEIGHT 150

using HValue = float;

typedef uint64_t Key;
#define MAXVEL 16
#define MAXDEPTH 16
Key Zorbistmypos[MAXWIDTH][MAXHEIGHT];
Key Zorbistrvpos[MAXWIDTH][MAXHEIGHT];
Key Zorbistmyvel[MAXVEL * 2][MAXVEL * 2];
Key Zorbistrvvel[MAXVEL * 2][MAXVEL * 2];

#define TT_HASH_BIT 18	// 15,16あたりが速い？
random_device rd;
//mt19937 mt(rd());
mt19937 mt(20180111);
uniform_int_distribution<int> rnd16(0, (1 << 16) - 1);

#ifdef DEBUGCOUNT
#define MAXDEBUGCOUNT 100
struct Debugcount {
	string name[MAXDEBUGCOUNT];
	int count[MAXDEBUGCOUNT];
	int totalcount[MAXDEBUGCOUNT];
	map<string, int> namemap;
	Debugcount() {
		memset(totalcount, 0, sizeof(int) * MAXDEBUGCOUNT);
		clearcount();
	}
	void clearcount() {
		memset(count, 0, sizeof(int) * MAXDEBUGCOUNT);
	}
	void print(const int num) {
		for (int i = 0; i < num && i < MAXDEBUGCOUNT ; i++) {
			cerr << name[i] << ": " << count[i] << " ";
		}
		cerr << endl;
	}
	void printtotal(const int num) {
		for (int i = 0; i < num && i < MAXDEBUGCOUNT; i++) {
			cerr << name[i] << ": " << totalcount[i] << " ";
		}
		cerr << endl;
	}
	void add(const int num) {
		if (num >= 0 && num < MAXDEBUGCOUNT) {
			count[num]++;
			totalcount[num]++;
		}
	}
	//void add(const string str) {
	//	if (namemap.count(str) > 0) {
	//		add(namemap[str]);
	//	}
	//}
	void setname(const int num, const string str) {
		if (num >= 0 && num < MAXDEBUGCOUNT) {
			name[num] = str;
			namemap[str] = num;
		}
	}
} dcount;
#define DNAME(num, name) dcount.setname(num, name);
#define DCLEAR dcount.clearcount();
#define DADD(num) dcount.add(num);
#define DPRINT(num) dcount.print(num);
#define DPRINTTOTAL(num) dcount.printtotal(num);
#else
#define DNAME(num, name)
#define DCLEAR
#define DADD(num) 
#define DPRINT(num)
#define DPRINTTOTAL(num)
#endif

// visual studio で開発する場合
#if defined(_MSC_VER)
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
// それ以外の場合
#else
#endif

using CVal = int32_t;

struct IntVec {
	CVal x, y;
	IntVec(CVal x = 0, CVal y = 0) : x(x), y(y) {};
	inline IntVec operator+(const IntVec &another) const {
		return IntVec(x + another.x, y + another.y);
	}
	inline bool operator==(const IntVec &another) const {
		return x == another.x && y == another.y;
	}
	inline bool operator!=(const IntVec &another) const {
		return !(*this == another);
	}
	inline bool operator<(const IntVec &another) const {
		return y != another.y ? y < another.y : x < another.x;
	}
};

typedef IntVec Point;

struct LineSegment {
	Point p1, p2;
	LineSegment() {};
	LineSegment(Point p1, Point p2) : p1(p1), p2(p2) {};
	bool goesThru(const Point &p) const;
	bool intersects(const LineSegment &l) const;
};

struct Polygon {
	vector<Point> plist;
	Polygon(Point p) {
		plist.push_back(p);
	}
	void push_back(Polygon &poly);
};

enum struct ObstState { UNKNOWN, OBSTACLE, NONE };


struct Course {
	int width, length;
	int vision;
	int thinkTime, stepLimit;
	int	visiony, maxvisiony;
	ObstState obstacle[MAXWIDTH][MAXHEIGHT];

	bool obstacled(const Point &from, const Point &to) const;
	Course(istream &in);
	void put(int y, const vector<int>& arr);
};

struct RaceState {
	int step;
	int timeLeft;
	Point position, oppPosition;
	IntVec velocity, oppVelocity;
	RaceState(int s, istream &in, Course &course);
};

struct Historydata {
	Point mpos;
	Point nbmpos;
	Point epos;
#ifdef RECORDHIST
	IntVec mvel;
	IntVec evel;
	IntVec mact;
	IntVec eact;
	Historydata(Point m, Point nb, Point e, IntVec mv, IntVec ev, IntVec ma, IntVec ea, double h) : mpos(m), nbmpos(nb), epos(e), mvel(mv), evel(ev), mact(ma), eact(ea) {};
#endif
//	Historydata(Point m, Point nb, Point e, IntVec mv, IntVec ev) : mpos(m), nbmpos(nb), epos(e), mvel(mv), evel(ev) {};
	Historydata(Point m, Point nb, Point e) : mpos(m), nbmpos(nb), epos(e) {};
	Historydata() {};
};

using History = vector<Historydata>;

#ifdef RECORDHIST
struct Hyouka {
	HValue hyouka;
	Hyouka() {};
	History hist;
	Hyouka(HValue h, History his) : hyouka(h), hist(his) {};
};
#else
using Hyouka = HValue;
#endif

#ifdef USE_TT
int8_t tt[1 << 24];

struct TT_DATA {
#ifdef RECORDHIST
	Hyouka		hyouka;
#else
	HValue		hyouka;
#endif
	int32_t		depth;
	Key			key;
};

#define TT_CLUSTERCOUNT (1 << TT_HASH_BIT)
#define TT_CLUSTERSIZE	4
struct TT_CLUSTER {
	TT_DATA data[TT_CLUSTERSIZE];
};

struct alignas(32) TT_TABLE {
	TT_CLUSTER table[TT_CLUSTERCOUNT];
#ifdef TT_COUNT
	int hitcount, nothitcount, conflictcount, dropcount;
	int totalhitcount, totalnothitcount, totalconflictcount, totaldropcount;
#endif
	// DISTANCETABLE から key に一致するキャッシュがあればそれを返す
	TT_DATA *findcache(const Key key, bool& found) {
		TT_CLUSTER *c = &table[key & (TT_CLUSTERCOUNT - 1)];
		for (int i = 0; i < TT_CLUSTERSIZE; i++) {
#ifdef RECORDHIST
			if (c->data[i].hyouka.hyouka == 0) {
#else
			if (c->data[i].hyouka == 0) {
#endif
				break;
			}
			if (c->data[i].key == key) {
				found = true;
#ifdef TT_COUNT
				hitcount++;
				totalhitcount++;
				if (i > 0) {
					conflictcount++;
					totalconflictcount++;
				}
#endif
				return &c->data[i];
			}
			}
		found = false;
		for (int i = 0; i < 4; i++) {
#ifdef RECORDHIST
			if (c->data[i].hyouka.hyouka == 0) {
#else
			if (c->data[i].hyouka == 0) {
#endif
#ifdef TT_COUNT
				nothitcount++;
				totalnothitcount++;
#endif
				return &c->data[i];
			}
			}
		for (int i = TT_CLUSTERSIZE - 1; i > 0; i--) {
			c->data[i] = c->data[i - 1];
		}
#ifdef TT_COUNT
		dropcount++;
		totaldropcount++;
		nothitcount++;
		totalconflictcount++;
#endif
		return &c->data[0];
		}
#ifdef TT_COUNT
	TT_TABLE() : totalhitcount(0), totalnothitcount(0), totalconflictcount(0), totaldropcount(0) {
#else
	TT_TABLE() {
#endif
		//	table = (TT_CLUSTER *)malloc(sizeof(TT_CLUSTER) * TT_CLUSTERCOUNT);
		clear();
	}
	void clear() {
#ifdef TT_COUNT
		hitcount = 0;
		nothitcount = 0;
		dropcount = 0;
		conflictcount = 0;
#endif
		memset(&table[0], 0, sizeof(TT_CLUSTER) * TT_CLUSTERCOUNT);
	}
#ifdef TT_COUNT
	void dispttcount() {
		cerr << "hit " << hitcount << " nothit " << nothitcount << " confilct " << conflictcount << " drop " << dropcount << endl;
	}
	void disptotalttcount() {
		cerr << "hit " << totalhitcount << " nothit " << totalnothitcount << " confilct " << totalconflictcount << " drop " << totaldropcount << endl;
	}
#endif
	};

TT_TABLE ttab;

#endif


// 時間を計測するクラス
class Timer {
public:
	Timer() {
		reset();
	}
	// 時間をリセットする
	void reset() {
		totaltime = 0;
		starttime = chrono::high_resolution_clock::now();
		stopflag = false;
	}
	void stop() {
		if (stopflag == false) {
			totaltime += chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - starttime).count();
			stopflag = true;
		}
	}
	void start() {
		if (stopflag == true) {
			starttime = chrono::high_resolution_clock::now();
			stopflag = false;
		}
	}
	// 経過時間を取得する
	int gettime() const {
		return static_cast<int>(totaltime + (stopflag ? 0 : chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - starttime).count()) / 1000);
	}
	int64_t getmicrotime() const {
		return totaltime + (stopflag ? 0 : chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - starttime).count());
	}
private:
	chrono::high_resolution_clock::time_point starttime, stoptime;
	bool stopflag;
	int64_t totaltime = 0;
};

void initzorbist();

