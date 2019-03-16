#include <map>
#include <queue>
#include <utility>
#include <iomanip>
#include <string>
#include <set>
//#include "raceState.hpp"
#include "samuraiCoding2017.hpp"



int START_SEARCH_DEPTH = 8;
int START_ENE_SEARCH_DEPTH = 4;

int MAX_SEARCH_DEPTH = 15;
int MAX_ENE_SEARCH_DEPTH = 4;

int SEARCH_DEPTH = START_SEARCH_DEPTH;
int ENE_SEARCH_DEPTH = START_ENE_SEARCH_DEPTH;

double MAX_SPEEDMUL = 0.5;
int MAX_SPEEDPLUS = 0;
int MAX_SPEED = 0;

bool calcnb = false;

bool changedepth = false;

const int MAX_SPEEDY = 8;
const int MIN_SPEEDY = 3;
// 9 5 

int maxspeedy;

int maxdepth;
int totaldepth;

#ifdef RECORDHIST
#ifdef DEBUG_DISPHYOUKA
const int debughiststep = 10;
const int debughistdepth = 1;
vector<IntVec> debugmove = {
	IntVec(-1, 0),
	IntVec(0, -1),
	IntVec(1, -1),
	IntVec(1, -1),
	IntVec(1, 1),
	IntVec(0, 0),
};
#endif
#endif
int step;

HValue placehyouka[MAXWIDTH][MAXHEIGHT];
// そのy座標の先が行き止まりかどうか。0：行き止まりではない。1:左の壁方向で行き止まり、2:右の壁方向で行き止まり。
int isleftorright[MAXWIDTH][MAXHEIGHT];

struct PlayerState {
	Point position;
	IntVec velocity;
	PlayerState(Point p, IntVec v) : position(p), velocity(v) {}
};

// 障害物との衝突チェック。
// fromは必ずフィールド内で、障害物が存在しない点であることが保証されているものとする。
bool Course::obstacled(const Point &from, const Point &to) const {
	// 調べる線分
	LineSegment m(from, to);
	const int &x1 = from.x;
	const int &y1 = from.y;
	const int &x2 = to.x;
	const int &y2 = to.y;

	// to がフィールド外の場合は true
	if (x2 < 0 || x2 >= width || y2 < 0) {
		return true;
	}
	
	// to に障害物があれば true
	if (obstacle[x2][y2] == ObstState::OBSTACLE) return true;

#ifdef USE_TT
	// ここに来た時点で、ルールよりx(0-19)は5ビット、y(0-100)は7ビットの範囲に収まるので、全部で24ビット
	int index = from.x | (from.y << 5) | (to.x << 12) | (to.y << 17);
	// 置換表に結果が入っていれば置換表の値を返す
	// 置換表は 0: 未チェック 1:true -1: false を表す
	if (tt[index] != 0) {
		DADD(D_TT1)
		return tt[index] > 0 ? true : false;
	}
	DADD(D_NTT1)
	
	//置換表には衝突するものとした値を入れておく
	tt[index] = 1;
#endif

	// 線分の正規ベクトルのx軸方向の値
	int xstep = x2 > x1 ? 1 : -1;
	// y軸が同じ場合は、x1<x<=x2 の各格子点を通るので、それらのうち一つでも障害物があれば true を返して終了
	if (y1 == y2) {
		// from == to であれば接触しないのでfalseを返して終了
		if (x1 == x2) {
#ifdef USE_TT
			tt[index] = -1;
#endif
			return false;
		}
		for (int x = x1 + xstep ; x != x2 ; x += xstep) {
			if (obstacle[x][y1] == ObstState::OBSTACLE) return true;
		}
		// そうでなければ交差しないので false を返して終了
#ifdef USE_TT
		tt[index] = -1;
#endif
		return false;
	}
	// y軸方向にも同じ処理を行う。ただしfrom = to はチェック済みなのでここではチェックしない
	int ystep = y2 > y1 ? 1 : -1;
	if (x1 == x2) {
		for (int y = y1 + ystep; y != y2; y += ystep) {
			if (obstacle[x1][y] == ObstState::OBSTACLE) return true;
		}
#ifdef USE_TT
		tt[index] = -1;
#endif
		return false;
	}

#ifdef MYOBED
	// from と to のx軸及び、y軸方向の差の絶対値を計算する
	const int dx = abs(x2 - x1);
	const int dy = abs(y2 - y1);
	// 傾きの絶対値が1の場合
	if (dx == dy) {
		for (int y = y1, x = x1; y != y2; ) {
			const int nx = x + xstep;
			const int ny = y + ystep;
			if ((obstacle[nx][ny] == ObstState::OBSTACLE) ||
				(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx][y] == ObstState::OBSTACLE)) {
				return true;
			}
			x = nx;
			y = ny;
		}
	}
	// 傾きの絶対値が1以下の場合
	else if (dx > dy) {
		int ycount = 0;
		int y = y1;
		for (int x = x1; x != x2; ) {
			const int nx = x + xstep;
			ycount += dy;
			int ny = y;
			const int ny2 = y + ystep;
			bool nextflag = false;
			if (ycount >= dx) {
				ny += ystep;
				ycount -= dx;
				nextflag = true;
			}
			// 次の点が格子上にある場合
			if (ycount == 0) {
				if ((obstacle[nx][ny2] == ObstState::OBSTACLE) ||
					(obstacle[x][ny2] == ObstState::OBSTACLE && obstacle[nx][y] == ObstState::OBSTACLE)) {
					return true;
				}
			}
			else if (nextflag == false) {
				if ((obstacle[nx][y] == ObstState::OBSTACLE && obstacle[nx][ny2] == ObstState::OBSTACLE) ||
					(obstacle[x][ny2] == ObstState::OBSTACLE && obstacle[nx][y] == ObstState::OBSTACLE) ||
					(obstacle[x][y] == ObstState::OBSTACLE && obstacle[nx][ny2] == ObstState::OBSTACLE)) {
					return true;
				}
			}
			else {
				int ny3 = ny2 + ystep;
				if ((obstacle[x][ny2] == ObstState::OBSTACLE && obstacle[nx][ny2] == ObstState::OBSTACLE) ||
					(obstacle[x][ny2] == ObstState::OBSTACLE && obstacle[nx][y] == ObstState::OBSTACLE) ||
					(obstacle[nx][ny2] == ObstState::OBSTACLE && obstacle[nx][ny3] == ObstState::OBSTACLE) ||
					(obstacle[x][ny3] == ObstState::OBSTACLE && obstacle[nx][ny2] == ObstState::OBSTACLE)) {
					return true;
				}
			}
			x = nx;
			y = ny;
		}
	}
	else {
		int xcount = 0;
		int x = x1;
		for (int y = y1; y != y2;) {
			int ny = y + ystep;
			xcount += dx;
			int nx = x;
			int nx2 = x + xstep;
			bool nextflag = false;
			if (xcount >= dy) {
				nx += xstep;
				xcount -= dy;
				nextflag = true;
			}
			// 次の点が格子上にある場合
			if (xcount == 0) {
				if ((obstacle[nx2][ny] == ObstState::OBSTACLE) ||
					(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx2][y] == ObstState::OBSTACLE)) {
					return true;
				}
			}
			else if (nextflag == false) {
				if ((obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx2][ny] == ObstState::OBSTACLE) ||
					(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx2][y] == ObstState::OBSTACLE) ||
					(obstacle[x][y] == ObstState::OBSTACLE && obstacle[nx2][ny] == ObstState::OBSTACLE)) {
					return true;
				}
			}
			else {
				int nx3 = nx2 + xstep;
				if ((obstacle[nx2][y] == ObstState::OBSTACLE && obstacle[nx2][ny] == ObstState::OBSTACLE) ||
					(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx2][y] == ObstState::OBSTACLE) || 
					(obstacle[nx2][ny] == ObstState::OBSTACLE && obstacle[nx3][ny] == ObstState::OBSTACLE) ||
					(obstacle[nx2][ny] == ObstState::OBSTACLE && obstacle[nx3][y] == ObstState::OBSTACLE)) {
					return true;
				}

			}
			x = nx;
			y = ny;
		}
	}
#else
	for (int y = y1; y != y2; y += ystep) {
		int ny = y + ystep;
		for (int x = x1; x != x2; x += xstep) {
			int nx = x + xstep;
			if ((obstacle[x][y] == ObstState::OBSTACLE && obstacle[nx][ny] == ObstState::OBSTACLE &&
				LineSegment(Point(x, y), Point(nx, ny)).intersects(m)) ||
				(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx][ny] == ObstState::OBSTACLE &&
					LineSegment(Point(x, ny), Point(nx, ny)).intersects(m)) ||
					(obstacle[nx][y] == ObstState::OBSTACLE && obstacle[nx][ny] == ObstState::OBSTACLE &&
						LineSegment(Point(nx, y), Point(nx, ny)).intersects(m)) ||
						(obstacle[x][ny] == ObstState::OBSTACLE && obstacle[nx][y] == ObstState::OBSTACLE &&
							LineSegment(Point(x, ny), Point(nx, y)).intersects(m))) {
				//				if (c1 == 635676) {
				//					cerr << "a " << x << "," << y << "," << nx << "," << ny << endl;
				//				}
				return true;
			}
		}
	}
#endif
#ifdef USE_TT
	tt[index] = -1;
#endif
	return false;
}

std::ostream& operator<<(std::ostream& out, const Point& p)
{
	return out << "(" << setw(2) << static_cast<int>(p.x) << ", " << setw(2) << static_cast<int>(p.y) << ")";
}

std::ostream& operator<<(std::ostream& out, const LineSegment& ls)
{
	return out << ls.p1 << " => " << ls.p2;
}

std::ostream& operator<<(std::ostream& out, const PlayerState& ps)
{
	return out << "{" << ps.position << ", " << ps.velocity << "}";
}

map<Point, int> bfsed;

static HValue cal(const PlayerState& me, const PlayerState& rv, History& hist, const int depth, const RaceState& rs, const Course& course)
{
	// calculating heuristic score here
	// HINT: this code use only y-axis, but we can do all kinds of things to win
	//	const auto de = decode(hist, course);
	HValue val = 0;
	//	val += (SEARCH_DEPTH - depth) * de.second;
	//	val += de.first;

#ifdef DEBUGCOUNT
	if (depth > maxdepth) {
		maxdepth = depth;
	}
#endif

	val = 0;
#if 1
	for (int i = 0; i < depth; i++) {
			//	int i = depth - 1;
		val /= 10;
		val += placehyouka[hist[i].mpos.x][hist[i].mpos.y] * 10;
		if (hist[i].epos.y >= 0) {
			val -= placehyouka[hist[i].epos.x][hist[i].epos.y];
		}
#ifdef CALCNB
		if (calcnb) {
			val += placehyouka[hist[i].mpos.x][hist[i].nbmpos.y];
		}
#endif
	}
#else
	val += placehyouka[hist[0].mpos.x][hist[0].mpos.y] * 10;
	val -= placehyouka[hist[0].epos.x][hist[0].epos.y];
	if (depth > 0) {
		val /= 10;
		val += placehyouka[hist[depth - 1].mpos.x][hist[depth - 1].mpos.y] * 10;
		val -= placehyouka[hist[depth - 1].epos.x][hist[depth - 1].epos.y];
	}
#endif
	if (depth < SEARCH_DEPTH) {
		hist[depth].mpos = Point(-1, -1);
	}
	val += (SEARCH_DEPTH - depth) * 10000;
	return val;
}

const HValue INF = static_cast<HValue>(1e20);

using State = pair<int, pair<PlayerState, PlayerState>>;


using Action = IntVec;

Action bestaction(0, 0);

// this function(pseudo alpha-beta algorithm) attempt to prevent enemy's move
static Hyouka alpha_beta(const RaceState& rs, const Course& course, const PlayerState& me, const PlayerState& rv,
	History &hist, int depth = 0, HValue alpha = -INF, HValue beta = INF)
{
#ifdef RECORDHIST
	History bhist(SEARCH_DEPTH);
	History bhist2(SEARCH_DEPTH);
	bool cflag = false;
	bool cflag2 = false;
#endif

#ifndef USE_ALPHABETA
	alpha = -INF;
	beta = INF;
#endif

	DADD(D_AB1)

	Key z = Zorbistmypos[me.position.x][me.position.y] ^ Zorbistmyvel[me.velocity.x + 16][me.velocity.y + 16] ^ Zorbistrvpos[rv.position.x][rv.position.y] ^ Zorbistrvvel[rv.velocity.x + 16][rv.velocity.y + 16];
	bool found;
	TT_DATA *tdata = ttab.findcache(z, found);
	if (found && tdata->depth <= depth) {
		return tdata->hyouka;
	}
	DADD(D_AB2)

	if (depth == SEARCH_DEPTH || me.position.y >= course.length || me.position.y >= course.visiony + 1) {
#ifdef RECORDHIST
		Hyouka ret(cal(me, rv, hist, depth, rs, course), hist);
		return ret;
#else
		Hyouka ret = cal(me, rv, hist, depth, rs, course);
// todo! ここ置換表に入れないほうが速いケースが多い！？要検証
//		tdata->key = z;
//		tdata->depth = depth;
//		tdata->hyouka = ret;
		return ret;
#endif
	}

	for (CVal my = 1; -1 <= my; --my) {
		// limit velocity
#ifdef MYAB2
		if (me.velocity.y + my > maxspeedy || me.velocity.y + my < -1) {
#else
		if (me.velocity.y + my > course.vision / 2) {
#endif
			continue;
		}

		for (CVal mx = -1; mx <= 1; ++mx) {
			HValue gamma = beta;
#ifdef MYAB 
			PlayerState nextMe = me;
			nextMe.velocity.x += mx;
			// 枝刈。x軸方向の速度が maxspeed を超えていた場合
			//if (nextMe.velocity.x < -maxspeedx || nextMe.velocity.x > maxspeedx) {
			//	continue;
			//}
			const int islorr = isleftorright[me.position.x][me.position.y];
			if ((islorr == 1 && nextMe.velocity.x <= -2 && mx <= 0) ||
				(islorr == 2 && nextMe.velocity.x >= 2 && mx >= 0)) {
				DADD(D_O2)
				continue;
			}
			nextMe.velocity.y += my;
			nextMe.position.x += nextMe.velocity.x;
			nextMe.position.y += nextMe.velocity.y;

//		if ((nextMe.position.x < 0 && mx <= 0) || (nextMe.position.x >= course.width && mx >= 0)) {
			// 枝刈。x軸方向でコースアウトしていた場合、もしくは次の移動でx軸方向にコースアウトする場合に、衝突方向と逆方向に加速していなかった場合
//			if (((nextMe.position.x < 0 || (nextMe.position.x + nextMe.velocity.x + 1 < 0)) && mx <= 0) || ((nextMe.position.x >= course.width || (nextMe.position.x + nextMe.velocity.x - 1 >= course.width)) && mx >= 0)) {
// todo! この枝刈やる？
			static const int vtable[MAXWIDTH + 10] = { -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -9, -9 };
//			if (((nextMe.position.x < 0 || (nextMe.position.x >= 0 && vtable[nextMe.position.x] >= nextMe.velocity.x)) && mx <= 0) ||
//				((nextMe.position.x >= course.width || (nextMe.position.x < course.width && -vtable[course.width - 1 - nextMe.position.x] <= nextMe.velocity.x)) && mx >= 0)) {
			if (((nextMe.position.x < -1 || (nextMe.position == -1 && nextMe.velocity < -1) || (nextMe.position.x >= 0 && vtable[nextMe.position.x] >= nextMe.velocity.x)) && mx <= 0) ||
				((nextMe.position.x > course.width || (nextMe.position.x == course.width && nextMe.velocity.x > 1) || (nextMe.position.x < course.width && -vtable[course.width - 1 - nextMe.position.x] <= nextMe.velocity.x)) && mx >= 0)) {
					DADD(D_O1);
				continue;
			}
			const LineSegment myMove(me.position, nextMe.position);
			bool obstacled = false;
			if (course.obstacled(me.position, nextMe.position)) {
				nextMe.position = me.position;
				obstacled = true;
				DADD(D_OB1)
			}
			// 障害物に阻まれた場合、加速も減速もしなければ、状況が変わらないので除く
			// todo!! 相手に邪魔されて動けない場合、何もしないほうが良いケースがあるかも？
			if (obstacled && mx == 0 && my == 0) {
				continue;
			}
			if (obstacled && ((islorr == 1 && nextMe.velocity.x < 0 && mx <= 0) ||
				(islorr == 2 && nextMe.velocity.x > 0 && mx >= 0))) {
				DADD(D_O2)
				continue;
			}

			// rv.position.y < 0 の場合は、視界外なので無視する
			if (depth < ENE_SEARCH_DEPTH && rv.position.y >= 0)  {
				Point nextMepositionbak = nextMe.position;
				for (CVal ey = 1; -1 <= ey; --ey) {
#ifdef MYAB2
					// 敵の場合は、y軸方向の速度がすでにマイナスになっている場合があるので、さらにey <= 0 の場合のみ除くことにする
					if ((rv.velocity.y + ey > maxspeedy && ey >= 0) || (rv.velocity.y + ey < -1 && ey <= 0)) {
						continue;
					}
#endif
					for (CVal ex = -1; ex <= 1; ++ex) {
						bool stopped = obstacled;
						nextMe.position = nextMepositionbak;
						if (!stopped && myMove.goesThru(rv.position)) {
							nextMe.position = me.position;
							stopped = true;
							DADD(D_OB2);
						}
						PlayerState nextRv = rv;
						nextRv.velocity.x += ex;
						// これ大丈夫？
						//if ((nextRv.velocity.x < -maxspeedx && ex <= 0) || (nextRv.velocity.x > maxspeedx && ex >= 0)) {
						//	continue;
						//}
						const int islorr = isleftorright[rv.position.x][rv.position.y];
						if ((islorr == 1 && nextRv.velocity.x <= -2 && ex <= 0) ||
							(islorr == 2 && nextRv.velocity.x >= 2 && ex >= 0)) {
							DADD(D_O2)
							continue;
						}
						nextRv.velocity.y += ey;
						nextRv.position.x += nextRv.velocity.x;
						nextRv.position.y += nextRv.velocity.y;
						//						if ((nextRv.position.x < 0 && ex <= 0) || (nextRv.position.x >= course.width && ex >= 0)) {
//						if (((nextRv.position.x < 0 || (nextRv.position.x + nextRv.velocity.x + 1 < 0)) && ex <= 0) || ((nextRv.position.x >= course.width || (nextRv.position.x + nextRv.velocity.x - 1 >= course.width)) && ex >= 0)) {
//						if (((nextRv.position.x < 0 || (nextRv.position.x >= 0 && vtable[nextRv.position.x] >= nextRv.velocity.x)) && ex <= 0) ||
//							((nextRv.position.x >= course.width || (nextRv.position.x < course.width && -vtable[course.width - 1 - nextRv.position.x] <= nextRv.velocity.x)) && ex >= 0)) {
						if (((nextRv.position.x < -1 || (nextRv.position == -1 && nextRv.velocity < -1) || (nextRv.position.x >= 0 && vtable[nextRv.position.x] >= nextRv.velocity.x)) && ex <= 0) ||
							((nextRv.position.x > course.width || (nextRv.position.x == course.width && nextRv.velocity.x > 1) || (nextRv.position.x < course.width && -vtable[course.width - 1 - nextRv.position.x] <= nextRv.velocity.x)) && ex >= 0)) {
								DADD(D_O1);
							continue;
						}
						const LineSegment enMove(rv.position, nextRv.position);
						bool eneobstacled = false;
						if (rv.position.y >= course.length
							|| (eneobstacled = course.obstacled(rv.position, nextRv.position))
							|| enMove.goesThru(me.position)) {
							nextRv.position = rv.position;
							stopped = true;
							DADD(D_OB3)
						}

						// 障害物に阻まれた場合、加速も減速もしなければ、状況が変わらないので除く
						// todo!! 相手に邪魔されて動けない場合、何もしないほうが良いケースがあるかも？
						if (eneobstacled && ex == 0 && ey == 0) {
							continue;
						}
						if (eneobstacled && ((islorr == 1 && nextRv.velocity.x < 0 && ex <= 0) ||
							(islorr == 2 && nextRv.velocity.x > 0 && ex >= 0))) {
							DADD(D_O2)
								continue;
						}

						bool intersected = false;
						if (!stopped && myMove.intersects(enMove)) {
							DADD(D_OB4)
								if (me.position.y != rv.position.y) {
									if (me.position.y < rv.position.y) {
										nextRv.position = rv.position;
									}
									else {
										nextMe.position = me.position;
										intersected = true;
									}
								}
								else if (me.position.x != rv.position.x) {
									if (me.position.x < rv.position.x) {
										nextRv.position = rv.position;
									}
									else {
										nextMe.position = me.position;
										intersected = true;
									}
								}
						}
#ifdef RECORDHIST
						//hist[depth] = Historydata(Point(nextMe.position.x, min(course.length, nextMe.position.y)),
						//	Point(nextRv.position.x, min(course.length, nextRv.position.y)),
						//	nextMe.velocity, nextRv.velocity, IntVec(mx, my), IntVec(ex, ey), 0);
						if (intersected) {
							hist[depth] = Historydata(nextMe.position, nextMepositionbak, nextRv.position,
								nextMe.velocity, nextRv.velocity, IntVec(mx, my), IntVec(ex, ey), 0);
						}
						else {
							hist[depth] = Historydata(nextMe.position, nextMe.position, nextRv.position,
								nextMe.velocity, nextRv.velocity, IntVec(mx, my), IntVec(ex, ey), 0);
						}
#else
						if (intersected) {
//							hist[depth] = Historydata(nextMe.position, nextMepositionbak, nextRv.position, nextMe.velocity, nextRv.velocity);
							hist[depth] = Historydata(nextMe.position, nextMepositionbak, nextRv.position);
						}
						else {
//							hist[depth] = Historydata(nextMe.position, nextMe.position, nextRv.position, nextMe.velocity, nextRv.velocity);
							hist[depth] = Historydata(nextMe.position, nextMe.position, nextRv.position);
						}
#endif

						auto res = alpha_beta(rs, course, nextMe, nextRv, hist, depth + 1, alpha, gamma);
#ifdef RECORDHIST
#ifdef DEBUG_DISPHYOUKA
						if (debughiststep == step && depth == debughistdepth) {
							bool flag = true;
							for (int i = 0; i < debughistdepth; i++) {
								if (debugmove[i] != res.hist[i].mact) {
									flag = false;
									break;
								}
							}
							if (flag) {
								int i = 0;
								cerr << "debug res " << mx << "," << my << "," << ex <<"," << ey << endl;
								for (auto ite = res.hist.begin(); ite != res.hist.end(); ++ite) {
									cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
									i++;
								}
								i = 0;
								for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
									cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
									i++;
								}
								fprintf(stderr, "hyouka %f %f %f %f\n", alpha, beta, gamma, res.hyouka);
								double c = cal(me, rv, bhist, SEARCH_DEPTH, rs, course);
								fprintf(stderr, "hyouka %f\n", c);

							}
						}
#endif
						if (res.hyouka < gamma) {
							gamma = res.hyouka;
#else
						if (res < gamma) {
							gamma = res;
#endif

#ifdef RECORDHIST

//							if (alpha < gamma) {
								bhist = res.hist;
								cflag = true;
//								bhist = hist;
//							}
							//if (alpha < gamma) {
							//	if (depth <= 0) {
							//		cerr << "b " << mx << "," << my << " " << ex << "," << ey << "," << depth << endl;
							//		fprintf(stderr, "%I64d %I64d %I64d\n", alpha, beta, gamma);
							//		for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
							//			cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
							//			fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
							//		}
							//		History h(1);
							//		h[0] = Historydata(Point(nextMe.position.x, min(course.length, nextMe.position.y)),
							//			Point(nextRv.position.x, min(course.length, nextRv.position.y)),
							//			nextMe.velocity, nextRv.velocity, IntVec(mx, my), IntVec(ex, ey), 0);
							//		//for (auto ite = hist.begin(); ite != hist.end(); ++ite) {
							//		//	cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
							//		//	fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
							//		//}
							//		for (auto ite = h.begin(); ite != h.end(); ++ite) {
							//			cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
							//			fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
							//		}
							//	}
							//}
#endif
						}
						//					hist[depth] = Historydata(Point(0, 0), Point(0, 0));
#ifdef USE_ALPHABETA
						if (alpha >= gamma) {
							gamma = alpha;
							goto END_ENEMY_TURN;
						}
#endif
					}
				}
			}
			else {
#ifdef RECORDHIST
				hist[depth] = Historydata(nextMe.position, nextMe.position,
					rv.position,
					nextMe.velocity, IntVec(-1, -1), IntVec(mx, my), IntVec(-1, -1), 0);
#else
//				hist[depth] = Historydata(nextMe.position, nextMe.position,
//					rv.position, nextMe.velocity, IntVec(-1, -1));
				hist[depth] = Historydata(nextMe.position, nextMe.position,
					rv.position);
#endif
				auto res = alpha_beta(rs, course, nextMe, rv, hist, depth + 1, alpha, gamma);

#ifdef RECORDHIST
				gamma = res.hyouka;
#else
				gamma = res;
#endif

#ifdef RECORDHIST

//					if (alpha < gamma) {
					//	if (depth == 1 && step == 2) {
					//		fprintf(stderr, "aaa %f %f %f\n", res.hyouka, alpha, gamma);
					//	}
						bhist = res.hist;
						cflag = true;
//						bhist = hist;
//					}
					//if (alpha < gamma) {
					//	if (depth <= 0) {
					//		cerr << "b " << mx << "," << my << " " << ex << "," << ey << "," << depth << endl;
					//		fprintf(stderr, "%I64d %I64d %I64d\n", alpha, beta, gamma);
					//		for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
					//			cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
					//			fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
					//		}
					//		History h(1);
					//		h[0] = Historydata(Point(nextMe.position.x, min(course.length, nextMe.position.y)),
					//			Point(nextRv.position.x, min(course.length, nextRv.position.y)),
					//			nextMe.velocity, nextRv.velocity, IntVec(mx, my), IntVec(ex, ey), 0);
					//		//for (auto ite = hist.begin(); ite != hist.end(); ++ite) {
					//		//	cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
					//		//	fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
					//		//}
					//		for (auto ite = h.begin(); ite != h.end(); ++ite) {
					//			cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact;
					//			fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
					//		}
					//	}
					//}
#endif
			}
#else
			for (int ey = 1; -1 <= ey; --ey) {
				for (int ex = -1; ex <= 1; ++ex) {
					PlayerState nextMe = me;
					nextMe.velocity.x += mx;
					nextMe.velocity.y += my;
					nextMe.position.x += nextMe.velocity.x;
					nextMe.position.y += nextMe.velocity.y;
					const LineSegment myMove(me.position, nextMe.position);
					PlayerState nextRv = rv;
					nextRv.velocity.x += ex;
					nextRv.velocity.y += ey;
					nextRv.position.x += nextRv.velocity.x;
					nextRv.position.y += nextRv.velocity.y;
					const LineSegment enMove(rv.position, nextRv.position);
					bool stopped = false;
					//					cerr << "a31 ";
#ifdef DEBUG
					if (step == checkstep) {
						cerr << depth << "," << me.position.x << "," << me.position.y << "," << me.velocity.x << "," << me.velocity.y << "," << rv.position.x << "," << rv.position.y << "," << rv.velocity.x << "," << rv.velocity.y << endl;
						cerr << mx << "," << my << "," << ex << "," << ey << endl;
					}
#endif
					if (course.obstacled(me.position, nextMe.position)
						|| myMove.goesThru(rv.position)) {
						nextMe.position = me.position;
						stopped |= true;
						DADD(D_OB1)
#ifdef DEBUG
							if (step == checkstep) {
								cerr << "x1" << endl;
							}
#endif
					}
					//					cerr << "a32";
					if (rv.position.y >= course.length
						|| course.obstacled(rv.position, nextRv.position)
						|| enMove.goesThru(me.position)) {
						nextRv.position = rv.position;
						stopped |= true;
						DADD(D_OB2)
#ifdef DEBUG
							if (step == checkstep ) {
								cerr << "x2" << endl;
							}
#endif
					}
					if (!stopped && myMove.intersects(enMove)) {
						DADD(D_OB3)
#ifdef DEBUG
							if (step == checkstep) {
								cerr << "x3" << endl;
							}
#endif
						if (me.position.y != rv.position.y) {
							if (me.position.y < rv.position.y) {
								nextRv.position = rv.position;
							}
							else {
								nextMe.position = me.position;
							}
						}
						else if (me.position.x != rv.position.x) {
							if (me.position.x < rv.position.x) {
								nextRv.position = rv.position;
							}
							else {
								nextMe.position = me.position;
							}
						}
					}
					hist[depth] = make_pair(
						make_pair(
							nextMe.position.x,
							min(course.length, nextMe.position.y)),
						make_pair(
							nextRv.position.x,
							min(course.length, nextRv.position.y))
					);

					auto res = alpha_beta(rs, course, nextMe, nextRv, hist, depth + 1, alpha, gamma);
					hist[depth] = make_pair(make_pair(0, 0), make_pair(0, 0));
					if (res.first < gamma) {
						gamma = res.first;
					}
					if (alpha >= gamma) {
						gamma = alpha;
						goto END_ENEMY_TURN;
					}
				}
			}
#endif
		END_ENEMY_TURN:
#ifdef RECORDHIST
#ifdef DEBUG_DISPHYOUKA
			if (debughiststep == step && depth == debughistdepth) {
				bool flag = true;
				for (int i = 0; i < debughistdepth; i++) {
					if (debugmove[i] != bhist[i].mact) {
						flag = false;
						break;
					}
				}
				if (flag) {
					int i = 0;
					cerr << "debug hyouka " << mx << "," << my << endl;
					for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
						cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
						i++;
					}
					fprintf(stderr, "hyouka %f %f %f\n", gamma, alpha, beta);
					double c = cal(me, rv, bhist, SEARCH_DEPTH, rs, course);
					fprintf(stderr, "hyouka %f\n", c);

				}
			}
#endif
#endif
			if (alpha < gamma) {
				alpha = gamma;
				if (depth == 0) {
					bestaction = { mx,my };
#ifdef RECORDHIST
					//cerr << "bestact " << bestaction << endl;
					//fprintf(stderr, "besthyouka %f\n", gamma);
					//for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
					//	cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
					//}
#endif
				}
				//if (depth <= 0) {
				//	myBestAction = { mx, my };
				//	cerr << "a " << mx << "," << my << "," << depth << endl;
				//	fprintf(stderr, "%I64d %I64d\n", alpha, beta);
				//}
#ifdef RECORDHIST
				//cerr << "x " << bhist[0].first.first << "," << bhist[0].first.second << "," << flag << ", d:" << depth <<  endl;
				if (alpha < beta) {
//					besthist = bhist;
					bhist2 = bhist;
					cflag2 = true;
					//if (step == 13) {
					//	fprintf(stderr,"ccc %f %f\n", alpha, beta);
					//}
//					if (bhist2[10].mpos.x > 90) {
//						cerr << "error xx!! " << bhist2[10].mpos.x << "," << bhist2[10].mpos.y << "," << depth << endl;
//					}
					if (cflag == false) {
						cerr << "error yy !! " << mx << "," << my << "," << depth << endl;
					}
//					if (depth == 0) {
//						besthist = bhist;
//					}
					//	hist[depth] = make_pair(
					//		make_pair(
					//			nextMe.position.x,
					//			min(course.length, nextMe.position.y)),
					//		make_pair(0, 0));
					//	besthist = hist;
					//	for (int i = 0; i < SEARCH_DEPTH; i++) {
					//		besthist[i].second = bhist[i].second;
					//	}
					}
//				}
#endif
//				myBestAction = { mx, my };
			}
			if (alpha >= beta) {
#ifdef RECORDHIST
#ifdef DEBUG_DISPHYOUKA
				if (debughiststep == step && depth == debughistdepth) {
					bool flag = true;
					for (int i = 0; i < debughistdepth; i++) {
						if (debugmove[i] != bhist[i].mact) {
							flag = false;
							break;
						}
					}
					if (flag) {
						int i = 0;
						cerr << "final debug hyouka " << mx << "," << my << endl;
						for (auto ite = bhist.begin(); ite != bhist.end(); ++ite) {
							cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
							i++;
						}
						fprintf(stderr, "hyouka %f %f %f\n", gamma, alpha, beta);
						double c = cal(me, rv, bhist, SEARCH_DEPTH, rs, course);
						fprintf(stderr, "hyouka %f\n", c);

					}
			}
#endif
//				if (cflag == false) {
//					bhist2[10].mpos = Point(98, depth);
//				}
				Hyouka ret(beta, bhist2);
				tdata->key = z;
				tdata->depth = depth;
				tdata->hyouka = ret;
				return ret;
#else
				tdata->key = z;
				tdata->depth = depth;
				tdata->hyouka = beta;
				return beta;
#endif
			}
		}
	}
#ifdef RECORDHIST
//#if true
#ifdef DEBUG_DISPHYOUKA
	if (debughiststep == step && depth == debughistdepth) {
//	if (step == 13 && depth == 6) {
		bool flag = true;
		for (int i = 0; i < debughistdepth; i++) {
			if (debugmove[i] != bhist2[i].mact) {
				flag = false;
				break;
			}
		}
		if (flag) {
			int i = 0;
			cerr << "final debug hyouka " << (int)cflag << "," << (int)cflag2 << endl;
			for (auto ite = bhist2.begin(); ite != bhist2.end(); ++ite) {
				cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
				i++;
			}
			i = 0;
			for (auto ite = hist.begin(); ite != hist.end(); ++ite) {
				cerr << i << ":" << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
				i++;
			}
			fprintf(stderr, "hyouka %f %f\n", alpha, beta);
			double c = cal(me, rv, bhist, SEARCH_DEPTH, rs, course);
			fprintf(stderr, "hyouka %f\n", c);

		}
	}
#endif
//	if (cflag == false) {
//		bhist2[10].mpos = Point(99, depth);
//	}
	Hyouka ret(alpha, bhist2);
	tdata->key = z;
	tdata->depth = depth;
	tdata->hyouka = ret;
	return ret;
#else
	tdata->key = z;
	tdata->depth = depth;
	tdata->hyouka = alpha;
	return alpha;
#endif
}

static void bfs(const RaceState& rs, Course& course)
{
	bfsed.clear();
	queue<Point> queue;

	const int ymax = rs.position.y + course.vision;

	Timer t;
	for (int y = ymax + 1; y < MAXHEIGHT; y++) {
		for (int x = 0; x < course.width; x++) {
			placehyouka[x][y] = static_cast<HValue>(y * 10);
			isleftorright[x][y] = 0;
		}
	}
	for (int y = 0; y <= ymax; y++) {
		for (int x = 0; x < course.width; x++) {
			placehyouka[x][y] = static_cast<HValue>(-10000);
			isleftorright[x][y] = 0;
		}
	}
	
	for (int x = 0; x < course.width; ++x) {
		queue.push(Point(x, ymax + 1));
	}
	while (queue.size()) {
		Point p = queue.front();
		queue.pop();
//		cerr << p << "," << (int)queue.size() << endl;
		for (int dx = -3; dx <= 3; dx++) {
			for (int dy = -3; dy <= 3; dy++) {
				if (dx == 0 && dy == 0) {
					continue;
				}
				int nx = p.x + dx;
				int ny = p.y + dy;
				Point np(nx, ny);
				if (ny > course.length + course.vision || ny < rs.position.y - 10 || nx < 0 || nx >= course.width || course.obstacled(p, np)) {
					continue;
				}
				double len = sqrt((p.x - nx) * (p.x - nx) + (p.y - ny) * (p.y - ny));
				if (placehyouka[nx][ny] < placehyouka[p.x][p.y] - len * 10) {
//					cerr << p << np << (int)len <<  endl;
					placehyouka[nx][ny] = static_cast<HValue>(placehyouka[p.x][p.y] - len * 10);
					queue.push(np);
				}
			}
		}
	}

	for (int y = max(0, rs.position.y - course.vision); y < min(MAXHEIGHT, ymax + 10); y++) {
		for (int x = 0; x < course.width; x++) {
			if (course.obstacle[x][y] == ObstState::OBSTACLE) {
				continue;
			}
			int len = 1;
			while (1) {
				for (int x2 = x - len; x2 <= x + len; x2++) {
					if (x2 < 0 || x2 >= course.width) {
						goto LEN_END;
					}
					for (int i = -1; i < 1; i += 2) {
						if (i == 1 && (x2 == x - len || x2 == x + len)) {
							continue;
						}
						int y2 = y + (len - abs(x - x2)) * i;
						if (y2 < 0 || y2 >= course.length) {
							continue;
						}
						if (course.obstacle[x2][y2] == ObstState::OBSTACLE) {
							goto LEN_END;
						}
					}
				}
				len += 1;
			}
		LEN_END:
			placehyouka[x][y] += len;
		}
	}

	struct Obst {
		int l;
		int r;
		int minl;
		int maxr;
		Obst(int _l, int _r) : l(_l), r(_r), minl(_l), maxr(_r) {};
	};
	vector<Obst> olist;
	olist.push_back(Obst(-1, -1));
	olist.push_back(Obst(course.width, course.width));

	for (int y = ymax; y >= max(0, rs.position.y - course.vision); y--) {
		//cerr << "y " << y << endl;
		//for (auto it = olist.begin(); it != olist.end(); ++it) {
		//	cerr << "olist0 " << it->l << "," << it->r << "," << it->minl << "," << it->maxr << endl;
		//}
		for (auto it = olist.begin(); it != olist.end(); ++it) {
			//cerr << "olist " << it->l << "," << it->r << "," << it->minl << "," << it->maxr << endl;
			bool deleteflag = false;
			// この obst と次の obst の間に別の obst が出現していた場合の、左端の x の最小値
			int nextobstl = it->r + 2;
			// このobst のy行目の左端を調べる
			// obst の左端が 0 以下の時は、-1が左端
			if (it->l <= 0) {
				// なにもしない
			}
			// obstの左端の左に障害物場あれば、そこからつながる最も左の端の障害物が左端となる
			else if (course.obstacle[it->l - 1][y] == ObstState::OBSTACLE) {
				for (int x = it->l - 1; x >= 0; x--) {
					if (course.obstacle[x][y] == ObstState::OBSTACLE) {
						it->l = x;
					}
					else {
						break;
					}
				}
				// x 座標が 0 の場合はその左に壁が必ずあるので -1 とする
				if (it->l == 0) {
					it->l = -1;
				}
			}
			// そうでなく、 obst の左端に障害物があれば、そこが左の端となる
			else if (course.obstacle[it->l][y] == ObstState::OBSTACLE) {
				// 何も変更しなくて良い
			}
			// そうでない場合は、obstの左端の一つ右からobstの右端の一つ右まで障害物を探し、最も最初に見つかった場所が左の端となる
			else {
				bool found = false;
				for (int x = it->l + 1 ; x <= min(it->r + 1, course.width - 1); x++) {
					if (course.obstacle[x][y] == ObstState::OBSTACLE) {
						it->l = x;
						found = true;
						break;
					}
				}
				// 見つからなかった場合
				if (!found) {
					// 右端がコースの外の場合は、左端をコースの外に設定する
					if (it->r == course.width) {
						it->l = course.width;
					}
					// そうでなければ、このobstはなくなったので削除する
					else {
						deleteflag = true;
					}
				}
			}
			//cerr << "olist1 " << it->l << "," << it->r << "," << it->minl << "," << it->maxr << endl;
			// このobst のy行目の右端を調べる
			// obst の右端が course.width - 1 以上の時は、course.width が左端
			if (it->r >= course.width - 1) {
				// なにもしない
			}
			// obstの右端の右に障害物場あれば、そこからつながる最も右の端の障害物が右端となる
			else if (course.obstacle[it->r + 1][y] == ObstState::OBSTACLE) {
				for (int x = it->r + 1; x < course.width; x++) {
					if (course.obstacle[x][y] == ObstState::OBSTACLE) {
						it->r = x;
						nextobstl = x + 2;
					}
					else {
						break;
					}
				}
				// x 座標が course.width - 1 の場合はその右に壁が必ずあるので course.width とする
				if (it->r == course.width - 1) {
					it->r = course.width;
				}
			}
			// そうでなく、 obst の右端に障害物があれば、そこが右の端となる
			else if (it->r >= 0 && course.obstacle[it->r][y] == ObstState::OBSTACLE) {
				// 何も変更しなくて良い
			}
			// そうでない場合は、obstの右端の一つ左からobstの左端の一つ左まで障害物を探し、最も最初に見つかった場所が右の端となる
			else {
				bool found = false;
				for (int x = it->r - 1; x >= max(0, it->l - 1); x--) {
					if (course.obstacle[x][y] == ObstState::OBSTACLE) {
						it->r = x;
						found = true;
						break;
					}
				}
				// 見つからなかった場合
				if (!found) {
					// 左端がコースの外の場合は、右端をコースの外に設定する
					if (it->l == -1) {
						it->r = -1;
					}
					// そうでなければ、このobstはなくなったので削除する
					else {
						deleteflag = true;
					}
				}
			}
			//cerr << "olist2 " << it->l << "," << it->r << "," << it->minl << "," << it->maxr << "," << deleteflag << endl;
			// 削除しない場合
			if (!deleteflag) {
				// minl と maxr の更新
				if (it->l < it->minl) {
					it->minl = it->l;
					// minl が一つ前の maxr よりも小さい場合
					if (it != olist.begin() && (it - 1)->maxr >= it->minl) {
						(it - 1)->maxr = it->minl - 1;
					}
				}
				if (it->r > it->maxr) {
					it->maxr = it->r;
					// minr が次の minl よりも大きい場合
					if ((it + 1) != olist.end() && (it + 1)->minl <= it->maxr) {
						(it + 1)->minl = it->maxr + 1;
					}
				}
			}
			// このobstを削除して次へ
			else {

				if (it->minl <= (it - 1)->maxr) {
					(it - 1)->maxr = it->minl - 1;
				}
				//// minr が次の minl よりも大きい場合
				if ((it + 1) != olist.end() && (it + 1)->minl <= it->maxr) {
					(it + 1)->minl = it->maxr + 1;
				}

				olist.erase(it);
				it--;
			}
			//cerr << "olist3 " << it->l << "," << it->r << "," << it->minl << "," << it->maxr << "," << deleteflag << endl;
			int nextobstr;
			// 次の obst が存在する場合で、この obst と 次の obst の間に新しい obst が出現していれば、それを加える
			if ((it + 1) != olist.end()) {
				while (nextobstl < (it + 1)->l - 1) {
					//cerr << "n " << nextobstl << "," << (it + 1)->l - 1 << endl;
					bool found = false;
					for (int x = nextobstl; x < (it + 1)->l - 1; x++) {
						if (course.obstacle[x][y] == ObstState::OBSTACLE) {
							nextobstl = x;
							for (int x2 = x + 1; x2 < course.width; x2++) {
								if (course.obstacle[x2][y] != ObstState::OBSTACLE) {
									nextobstr = x2 - 1;
									break;
								}
								if (x2 == course.width - 1) {
									nextobstr = course.width;
								}
							}
							if (nextobstr < (it + 1)->l - 1) {
								found = true;
							}
							break;
						}
					}
					//cerr << "f " << found << "," << nextobstl << "," << nextobstr << endl;
					if (found) {
						//// minl が一つ前の maxr よりも小さい場合
						//if (it->maxr >= nextobstl) {
						//	it->maxr = nextobstl - 1;
						//}
						////// minr が次の minl よりも大きい場合
						//if ((it + 1) != olist.end() && (it + 1)->minl <= nextobstr) {
						//	(it + 1)->minl = nextobstr + 1;
						//}
						it = olist.insert(it + 1, Obst(nextobstl, nextobstr));
						
						nextobstl = nextobstr + 2;
						//for (auto it2 = olist.begin(); it2 != olist.end(); ++it2) {
						//	cerr << "olist0 " << it2->l << "," << it2->r << "," << it2->minl << "," << it2->maxr << endl;
						//}

					}
					else {
						break;
					}
				}
			}
		}
		int prevminl = -1;
		int prevmaxr = -1;
		for (auto it = olist.begin(); it != olist.end(); ++it) {
			for (int x = max(0, it->minl); x < it->l; x++) {
				if (x >= prevminl && x <= prevmaxr) {
					continue;
				}
				isleftorright[x][y] = 2;
			}
			for (int x = it->r + 1; x <= min(course.width - 1, it->maxr); x++) {
				if (x >= prevminl && x <= prevmaxr) {
					continue;
				}
				isleftorright[x][y] = 1;
			}
			prevminl = it->minl;
			prevmaxr = it->maxr;
		}
	}
}

int errcount = 0;
static Hyouka play(const RaceState& rs, Course& course) {

	bfs(rs, course);
	History hist(SEARCH_DEPTH);
	int count = 0;
	while (1) {
		DCLEAR
#ifdef USE_TT
			memset(tt, 0, 1 << 24);
#endif
		ttab.clear();
		auto p = alpha_beta(rs, course, { rs.position, rs.velocity }, { rs.oppPosition, rs.oppVelocity }, hist);
		// 評価値がマイナスの場合は、おそらくハッシュの衝突でうまくいっていない可能性があるので、Zorbistハッシュ値を振りなおしてやりなおす。
#ifdef RECORDHIST
		if (p.hyouka >= 100000000 || p.hyouka <= -10000000) {
#else
		if (p >= 100000000 || p <= -10000000) {
#endif		// If my player will be stuck, use greedy.
			//	bestaction = find_movable(rs, course);
			count++;
			errcount++;
			cerr << "error !! hyouka is -INF. play again (count = " << count << ")" << endl;
			initzorbist();
			if (count >= 3) {
#ifdef RECORDHIST
				History his(SEARCH_DEPTH);
				Hyouka h(0, his);
#else
				Hyouka h = 0;
#endif
				return h;
			}
		}
		else {
			DPRINT(DNUM);
			return p;
		}
	}
}

static int dot(int x1, int y1, int x2, int y2) {
	return x1 * x2 + y1 * y2;
}

static int cross(int x1, int y1, int x2, int y2) {
	return x1 * y2 - x2 * y1;
}

static int ccw(int x1, int y1, int x2, int y2, int x3, int y3) {
	return cross(x2 - x1, y2 - y1, x3 - x2, y3 - y2);
}

bool LineSegment::goesThru(const Point &p) const {
	int minx = min(p1.x, p2.x);
	if (p.x < minx) return false;
	int maxx = max(p1.x, p2.x);
	if (p.x > maxx) return false;
	int miny = min(p1.y, p2.y);
	if (p.y < miny) return false;
	int maxy = max(p1.y, p2.y);
	if (p.y > maxy) return false;
	return ccw(p1.x, p1.y, p2.x, p2.y, p.x, p.y) == 0 && dot(p1.x - p.x, p1.y - p.y, p2.x - p.x, p2.y - p.y) <= 0;
}

//int64_t ltime1 = 0;
//int64_t ltime2 = 0;
bool LineSegment::intersects(const LineSegment &l) const {
	DADD(D_IS1)

//	Timer t;
	int minx = min(p1.x, p2.x);
	int maxx = max(p1.x, p2.x);
	int minlx = min(l.p1.x, l.p2.x);
	int maxlx = max(l.p1.x, l.p2.x);
	if (maxx < minlx || maxlx < minx) {
//		ltime1 += t.getmicrotime();
		return false;
	}
	int miny = min(p1.y, p2.y);
	int maxy = max(p1.y, p2.y);
	int minly = min(l.p1.y, l.p2.y);
	int maxly = max(l.p1.y, l.p2.y);
	if (maxy < minly || maxly < miny) {
//		ltime1 += t.getmicrotime();
		return false;
	}
//	ltime1 += t.getmicrotime();
	DADD(D_IS2)
	int d1 = (p1.x - l.p1.x)*(l.p2.y - l.p1.y) - (p1.y - l.p1.y)*(l.p2.x - l.p1.x);
	int d2 = (p2.x - l.p1.x)*(l.p2.y - l.p1.y) - (p2.y - l.p1.y)*(l.p2.x - l.p1.x);
	if (d1*d2 > 0) {
		return false;
//		ltime2 += t.getmicrotime();
	}
	int d3 = (l.p1.x - p1.x)*(p2.y - p1.y) - (l.p1.y - p1.y)*(p2.x - p1.x);
	int d4 = (l.p2.x - p1.x)*(p2.y - p1.y) - (l.p2.y - p1.y)*(p2.x - p1.x);
	if (d3*d4 > 0) {
//		ltime2 += t.getmicrotime();
		return false;
	}
//	ltime2 += t.getmicrotime();
	return true;
}

RaceState::RaceState(int s, istream &in, Course &course) {
	step = s;
//	in >> step
	in	>> timeLeft
		>> position.x >> position.y
		>> velocity.x >> velocity.y
		>> oppPosition.x >> oppPosition.y
		>> oppVelocity.x >> oppVelocity.y;
	for (int y = position.y - course.vision;
		y <= position.y + course.vision;
		y++) {
		std::vector<int> arr;
		for (int x = 0; x != course.width; x++) {
			int o; in >> o;
			arr.push_back(o);
		}
		course.put(y, arr);
	}
}

void Polygon::push_back(Polygon &poly) {
	plist.insert(plist.end(), poly.plist.begin(), poly.plist.end());
}


Course::Course(istream &in) {
	in >> thinkTime >> stepLimit >> width >> length >> vision;
	maxvisiony = visiony = -1;
	for (int x = 0; x < MAXWIDTH; x++) {
		for (int y = 0; y < MAXHEIGHT; y++) {
//			if (y < length) {
				obstacle[x][y] = ObstState::UNKNOWN;
//			}
		}
	}
//	obstacle = Obstacle(width);
}

void Course::put(int y, const std::vector<int>& arr) {
	visiony = y;
	// y 行目の位置が、既に視界内の場合は何もしなくてよいので終了
	if (y <= maxvisiony) {
		return;
	}
	for (int x = 0; x < width; x++) {
		obstacle[x][y] = (arr[x] == 1) ? ObstState::OBSTACLE : ObstState::NONE;
	}
	// y行まで計算できたので visiony を更新する
	maxvisiony = y;
}

void initzorbist() {
	
	for (int y = 0; y < MAXHEIGHT; y++) {
		for (int x = 0; x < MAXWIDTH; x++) {
			Zorbistmypos[x][y] = 0;
			Zorbistrvpos[x][y] = 0;
			for (int i = 0; i < 4; i++) {
				Zorbistmypos[x][y] += static_cast<Key>(rnd16(mt));
				Zorbistrvpos[x][y] += static_cast<Key>(rnd16(mt));
				if (i != 3) {
					Zorbistmypos[x][y] <<= 16;
					Zorbistrvpos[x][y] <<= 16;
				}
			}
		}
	}
	for (int y = 0; y < MAXVEL * 2; y++) {
		for (int x = 0; x < MAXVEL * 2; x++) {
			Zorbistmyvel[x][y] = 0;
			Zorbistrvvel[x][y] = 0;
			for (int i = 0; i < 4; i++) {
				Zorbistmyvel[x][y] += static_cast<Key>(rnd16(mt));
				Zorbistrvvel[x][y] += static_cast<Key>(rnd16(mt));
				if (i != 3) {
					Zorbistmyvel[x][y] <<= 16;
					Zorbistrvvel[x][y] <<= 16;
				}
			}
		}
	}
}

int main(int argc, char *argv[]) {
	DNAME(D_AB1, "AB1")
	DNAME(D_AB2, "AB2")
	DNAME(D_IS1, "IS1")
	DNAME(D_IS2, "IS2")
	DNAME(D_TT1, "TT1")
	DNAME(D_NTT1, "NTT1")
	DNAME(D_OB1, "OB1")
	DNAME(D_OB2, "OB2")
	DNAME(D_OB3, "OB3")
	DNAME(D_OB4, "OB4")
	DNAME(D_O1, "O1")
	DNAME(D_O2, "O2")
	initzorbist();
	totaldepth = 0;

	int cnum = -1;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-sd") == 0 && argc > i + 1) {
			i++;
			START_SEARCH_DEPTH = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-sed") == 0 && argc > i + 1) {
			i++;
			START_ENE_SEARCH_DEPTH = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-md") == 0 && argc > i + 1) {
			i++;
			MAX_SEARCH_DEPTH = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-med") == 0 && argc > i + 1) {
			i++;
			MAX_ENE_SEARCH_DEPTH = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-msyi") == 0 && argc > i + 1) {
			i++;
			MAX_SPEEDMUL = static_cast<double>(stoi(argv[i], NULL)) / 100.0;
		}
		if (strcmp(argv[i], "-msyp") == 0 && argc > i + 1) {
			i++;
			MAX_SPEEDPLUS = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-msy") == 0 && argc > i + 1) {
			i++;
			MAX_SPEED = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-cnb") == 0 && argc > i + 1) {
			i++;
			if (stoi(argv[i], NULL, 10) != 0) {
				calcnb = true;
			}
		}
		if (strcmp(argv[i], "-c") == 0 && argc > i + 1) {
			i++;
			cnum = stoi(argv[i], NULL, 10);
		}
		if (strcmp(argv[i], "-cd") == 0 && argc > i + 1) {
			i++;
			changedepth = static_cast<bool>(stoi(argv[i], NULL, 10));
		}
	}

	SEARCH_DEPTH = START_SEARCH_DEPTH;
	ENE_SEARCH_DEPTH = START_ENE_SEARCH_DEPTH;

	Course course(cin);
	cout << 0 << endl;
	cout.flush();
	int totaltime = 0;
	cerr << "course data: width " << course.width << " height " << course.length << " vision " << course.vision << " maxstep " << course.stepLimit << " thinktime " << course.thinkTime << endl;
	maxspeedy = static_cast<int>(floor(course.vision * MAX_SPEEDMUL));
	if (maxspeedy > MAX_SPEEDY) {
		maxspeedy = MAX_SPEEDY;
	}
	else if (maxspeedy < MIN_SPEEDY) {
		maxspeedy = MIN_SPEEDY;
	}

	maxspeedy += MAX_SPEEDPLUS;
	if (MAX_SPEED != 0) {
		maxspeedy = MAX_SPEED;
	}
	if (maxspeedy > course.vision) {
		maxspeedy = course.vision;
	}
	int msy = maxspeedy;
	cerr << "start search depth " << START_SEARCH_DEPTH << " ene " << START_ENE_SEARCH_DEPTH << " max " << MAX_SEARCH_DEPTH << " ene " << MAX_ENE_SEARCH_DEPTH << " maxspeedmul " << MAX_SPEEDMUL << " maxspeedplus " << MAX_SPEEDPLUS << " maxspeedy " << maxspeedy << " calcnb " << calcnb << " changedepth " << changedepth << endl;

	while (true) {
		Timer t;
		int s;
		cin >> s;
		if (s < 0) {
			break;
		}
		RaceState rs(s, cin, course);
		cerr << "step " << rs.step << " " << rs.position << rs.velocity << " " << rs.oppPosition << rs.oppVelocity << " time left " << rs.timeLeft << endl;
		step = rs.step;
#ifdef DEBUG_DISPHYOUKA
		if (step != debughiststep) {
			continue;
		}
#endif

		totaltime += t.gettime();

#ifdef DEBUGCOUNT
		maxdepth = 0;
#endif
		Timer t2;
		if (course.visiony >= course.length) {
			maxspeedy = min(10, course.vision);
	}
		Hyouka hyouka = play(rs, course);
		totaltime += t2.gettime();

#ifdef RECORDHIST
		fprintf(stderr, "hyouka %f\n", hyouka.hyouka);
//		double c = cal({ rs.position, rs.velocity }, { rs.oppPosition, rs.oppVelocity }, hyouka.hist, SEARCH_DEPTH, rs, course);
//		fprintf(stderr, "hyouka2 %f\n", c);
#else 
		fprintf(stderr, "hyouka %f\n", hyouka);
#endif
		cout << static_cast<int>(bestaction.x) << ' ' << static_cast<int>(bestaction.y) << endl;
		cout.flush();

		cerr << static_cast<int>(bestaction.x) << ' ' << static_cast<int>(bestaction.y) << endl;
		int playtime = t2.gettime();
		cerr << "search depth " << SEARCH_DEPTH << " ene search depth " << ENE_SEARCH_DEPTH;
#ifdef DEBUGCOUNT
		totaldepth += maxdepth;
		cerr << " max seach depth " << maxdepth;
#endif
		cerr << endl;
		int stepleft = course.stepLimit - step;
		if (stepleft <= 0) {
			stepleft = 1;
		}
		cerr << "play time " << playtime << "ms" << " time left " << rs.timeLeft << " step left " << stepleft << " ave left time " << rs.timeLeft / stepleft << endl;
#ifdef CHANGEDEPTH
		if (changedepth) {
			if (playtime < rs.timeLeft / stepleft) {
				if (SEARCH_DEPTH == ENE_SEARCH_DEPTH || ENE_SEARCH_DEPTH == MAX_ENE_SEARCH_DEPTH) {
					if (SEARCH_DEPTH < MAX_SEARCH_DEPTH) {
						SEARCH_DEPTH++;
					}
				}
				else if (ENE_SEARCH_DEPTH < MAX_ENE_SEARCH_DEPTH) {
					ENE_SEARCH_DEPTH++;
				}
			}
			else {
				if (SEARCH_DEPTH > ENE_SEARCH_DEPTH && SEARCH_DEPTH > 1) {
					SEARCH_DEPTH--;
				}
				else if (ENE_SEARCH_DEPTH > 0) {
					ENE_SEARCH_DEPTH--;
				}
			}
		}
#endif
#if defined(USE_TT) && defined(TT_COUNT)
		ttab.dispttcount();
#endif
#ifdef RECORDHIST
		History& besthist = hyouka.hist;
#//		cerr << besthist.size() << endl;
		int i = 0;
		for (auto ite = besthist.begin(); ite != besthist.end(); ++ite) {
			cerr << ite->mpos << ite->mvel << ite->mact << " " << ite->epos << ite->evel << ite->eact << endl;
//			fprintf(stderr, "besthyouka %I64d\n", ite->hyouka);
			i++;
		}
		for (int y = rs.position.y + course.vision + 10; y >= max(0, rs.position.y - course.vision); y--) {
			cerr << setw(2) << y;
			if (y == rs.position.y) {
				cerr << "* ";
			}
			else {
				cerr << ": ";
			}
			for (int x = 0; x < course.width; x++) {
				Point p(x, y);
				//if (bfsed.count(p) == 0) {
				if (placehyouka[p.x][p.y] == -10000) {
						cerr << "   X ";
				}
				else {
//					cerr << setw(3) << bfsed[p];
					cerr << setw(4) << (int)placehyouka[p.x][p.y];
					bool flag = false;
					if (rs.position == p) {
						cerr << "A";
						continue;
					}
					if (rs.oppPosition == p) {
						cerr << "a";
						continue;
					}
					int i = 1;
					for (auto ite = besthist.begin(); ite != besthist.end(); ++ite) {
						if (ite->mpos == p) {
							cerr << (char)('A' + i);
							flag = true;
							break;
						}
						else if (ite->epos == p) {
							cerr << (char)('a' + i);
							flag = true;
							break;
						}
						i++;
					}
					if (!flag) {
						cerr << " ";
					}
				}
			}
			cerr << endl;
		}
		cerr << "    ";
		for (int x = 0; x < course.width; x++) {
			cerr << setw(4) << x << " ";
		}
		cerr << endl;

		for (int y = rs.position.y + course.vision + 10; y >= max(0, rs.position.y - course.vision); y--) {
			cerr << setw(2) << y;
			if (y == rs.position.y) {
				cerr << "* ";
			}
			else {
				cerr << ": ";
			}
			for (int x = 0; x < course.width; x++) {
				Point p(x, y);
				if (course.obstacle[p.x][p.y] == ObstState::OBSTACLE) {
					cerr << "   X ";
				}
				else if (course.obstacle[p.x][p.y] == ObstState::UNKNOWN) {
					cerr << "   ? ";
				}
				else if(isleftorright[p.x][p.y] == 0) {
					cerr << "   - ";
				}
				else if (isleftorright[p.x][p.y] == 1) {
					cerr << "   L ";
				}
				else if (isleftorright[p.x][p.y] == 2) {
					cerr << "   R ";
				}
			}
			cerr << endl;
		}
		cerr << "    ";
		for (int x = 0; x < course.width; x++) {
			cerr << setw(4) << x << " ";
		}
		cerr << endl;
#endif
	}
	double t1, t2;
	cin >> t1 >> t2;
	DPRINTTOTAL(DNUM)
	cerr << endl << "totaltime " << totaltime << "ms" << endl;
	cerr << "goal time " << t1 << " ms " << t2 << " ms" << endl;

#ifdef DEBUGCOUNT
	cerr << "total depth " << totaldepth << "," << msy << endl;
	if (cnum < 50) {
		fstream fs;
		fs.open("res.dat", ios::app);
		fs << cnum << "," << START_SEARCH_DEPTH << "," << START_ENE_SEARCH_DEPTH << "," << MAX_SEARCH_DEPTH << "," << MAX_ENE_SEARCH_DEPTH << "," << course.vision << "," << msy << "," << calcnb << "," << totaldepth << "," << totaltime << "," << errcount << "," << t1 << "," << t2 << endl;
		fs.close();
	}
#endif
	cerr << "error count " << errcount << endl;
#if defined(USE_TT) && defined(TT_COUNT)
	ttab.disptotalttcount();
#endif

//	cerr << "l1 " << (int)ltime1 << "," << (int)ltime2 << endl;
}
