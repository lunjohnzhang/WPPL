/*
 * node.cpp
 *
 * Purpose: Node
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "LNS/PIBT/node.h"

namespace LNS {

int Node::cntIndex = 0;

Node::Node(int _id) : id(_id), index(cntIndex) {
  ++cntIndex;
  pos = Vec2f(0, 0);
}

} // end namespace LNS