#pragma once

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

// 该文件实现了 Kuhn 和 Munkres 的算法，用于在完全加权二分图中找到最大权重匹配。
// 该算法也被称为 '匈牙利算法'。
// 完全二分加权图可以通过两个顶点集合定义，分别由 'i' 和 'j' 索引，并且有一个边权重函数 w(i, j)。
// "顶点标定" 由两个实值函数 lx(i) 和 ly(j) 定义，满足 lx(i) + ly(j) >= w(i, j) 对所有的 (i, j) 都成立。
// 顶点标定定义了一个 "等式子图"，包含所有 lx(i) + ly(j) == w(i, j) 的边。
// 算法基于一个定理，说明在等式子图中，针对顶点标定 (lx, ly) 的完全匹配也是原始图中的最大权重匹配。
// 算法的主要思想是交替执行两个步骤：
// 1) 增加等式子图中的匹配数，
// 2) 改进顶点标定以便于进行 (1)。
namespace {
constexpr double kEpsilon = 1e-10;// 精度容忍度，避免浮动误差影响匹配结果
// 定义一个具有以下特性的整数集合：
//   1. 记住元素加入的顺序，
//   2. 可以在常数时间内判断给定整数是否是集合的成员。
class OrderedSet {
 public:
  // 创建一个空集合，可以包含 0 到 n-1 之间的整数。
  explicit OrderedSet(int n) : M_(n) {}
  // 清空集合
  void clear() {
    int n = M_.size();
    M_.clear();
    M_.resize(n, false);
    I_.clear();
  }
  // 判断集合中是否包含元素 i
  bool has(int i) const { return M_[i]; }
  // 插入元素 i 到集合中
  void insert(int i) {
    if (!M_[i]) {
      M_[i] = true;
      I_.push_back(i);
    }
  }

  size_t size() const { return I_.size(); }

  // 返回第 idx 个成员
  int operator[](int idx) const { return I_[idx]; }
  // 判断两个 OrderedSet 是否相等
  bool operator==(const OrderedSet& s) const {
    if (size() != s.size()) return false;

    for (size_t k = 0; k < size(); ++k) {
      if (!s.has(I_[k])) return false;
    }
    return true;
  }

 private:
  std::vector<bool> M_;// 存储集合成员的布尔数组
  std::vector<int> I_;// 存储集合成员的列表
};

// 存储二分图中的部分或完整匹配
// 存储整数 [0, n-1] 的排列和逆排列
class Matching {
 public:
  explicit Matching(int n) : p_(n, -1), ip_(n, -1) {}
   // 向匹配中添加 (i, j)
  void Add(int i, int j) {
    p_[i] = j;
    ip_[j] = i;
  }
  // 获取与 i 匹配的 j，获取与 j 匹配的 i
  int GetJ(int i) const { return p_[i]; }
  int GetI(int j) const { return ip_[j]; }

  // 返回一个未匹配的 i，如果没有未匹配的 i，则返回 n
  int FindUnmatchedI() const {
    return std::find(p_.begin(), p_.end(), -1) - p_.begin();
  }
  // 将当前匹配复制到数组 p 中
  void CopyTo(int* p) {
    for (size_t i = 0; i < p_.size(); ++i) p[i] = p_[i];
  }

 private:
  std::vector<int> p_;// 用于存储每个 i 对应的 j
  std::vector<int> ip_;// 用于存储每个 j 对应的 i
};

// 向集合 N 中添加所有与 i 相连的 j 节点，这些节点与 i 在等式子图中存在边连接
void AddNeighbours(int i, int parent_index, int n, const double* edge,
                   const double* lx, const double* ly, OrderedSet* N,
                   std::vector<int>* parentN) {
  int numN = N->size();

  for (int j = 0; j < n; ++j) {
    if (std::abs(lx[i] + ly[j] - edge[i * n + j]) < kEpsilon) {// 如果 lx[i] + ly[j] == edge[i, j]，则认为 (i, j) 是等式子图中的边
      N->insert(j);
    }
  }
  parentN->insert(parentN->end(), N->size() - numN, parent_index);// 更新 parentN 记录每个 N 中元素的父节点
}

// 对于 S 中的每个顶点，查找它的邻居并加入 N 中
void FindNeighbours(const std::vector<int>& S, int n, const double* edge,
                    const double* lx, const double* ly, OrderedSet* N,
                    std::vector<int>* parentN) {
  N->clear();
  parentN->clear();

  for (size_t ii = 0; ii < S.size(); ++ii) {
    AddNeighbours(S[ii], ii, n, edge, lx, ly, N, parentN);
  }
}


// 改进可行性标定，使得存在至少一个从 S 中可达的等子图的顶点，且该顶点不在 T 中
void ImproveFeasibleLabeling(int n, const double* edge,
                             const std::vector<int>& S, const OrderedSet& T,
                             double* lx, double* ly) {
  double a = std::numeric_limits<double>::max();

  for (size_t ii = 0; ii < S.size(); ++ii) {
    int i = S[ii];
    for (int j = 0; j < n; ++j) {
      if (!T.has(j)) {
        a = std::min(a, lx[i] + ly[j] - edge[i * n + j]);
      }
    }
  }
  // 更新 lx 和 ly，使得新的标定满足等式子图条件
  for (size_t i = 0; i < S.size(); ++i) lx[S[i]] -= a;
  for (size_t j = 0; j < T.size(); ++j) ly[T[j]] += a;
}


// 在m中通过1来增加匹配数，通过在由S和T定义的交替树中从T[T.size(0-1] 叶节点到 S[0] 的路径
//parentT[k]是T[k]父节点的进入S的索引
void AugmentMatching(const std::vector<int>& S, const OrderedSet& T,
                     const std::vector<int>& parentT, Matching* m) {
  // 通过交替路径更新匹配，交替树的叶节点不应该匹配
  // The leaf of the alternating tree. It should be unmatched.
  int j = T[T.size() - 1];// T 的最后一个元素
  // 找到一个从叶节点到根节点的可替换路径，可替换路径是一种在匹配和未匹配之间可替换的边的路径
  for (int k = T.size() - 1; k >= 0; --k) {
    if (T[k] == j) {
      int ii = parentT[k];
      int i = S[ii];

      // (i, j) 是未匹配的边， 把它添加到匹配.
      // tmp_j 意思是如何找到一个可以再次上树的步骤
      int tmp_j = m->GetJ(i);
      m->Add(i, j);
      j = tmp_j;

      // 如果到达交替树的根节点，则匹配结束
      if (ii == 0) {
        // Checks that the root was unmatched when we got to it.
        // CHECK_EQ(tmp_j, -1);
        return;
      }
    }
  }

  // If we get here, we have a bug somewhere in this file...
  //   LOG(FATAL);
}

// 从集合 N 中选择一个不在 T 中的元素
// 该函数假设至少一个这样的函数
int PickY(const OrderedSet& N, const OrderedSet& T) {
  for (size_t j = 0; j < N.size(); ++j) {
    if (!T.has(N[j])) return j;
  }

  // If we get here, we shouldn't have called this function in the first place.
  //   LOG(FATAL);
  return -1;
}

// 更新最终的匹配结果，找出匹配的对和未匹配的行列
//costs的sizes是从m*n膨胀到n*n，作为一个原始的Hungarian requires
//该函数从完美匹配中提取真实的关联结果
void UpdateAssociatedResult(
    const size_t rows_size, const size_t cols_size, const double* edge,
    const int* match, std::vector<std::pair<size_t, size_t>>* associated_pairs,
    std::vector<size_t>* unassociated_rows,
    std::vector<size_t>* unassociated_cols) {
  const size_t num_vertices = std::max(rows_size, cols_size);
  for (size_t i = 0; i < num_vertices; ++i) {
    const size_t j = match[i];
    if (edge[i * num_vertices + j] > 0) {
      associated_pairs->push_back(std::make_pair(i, j));
    } else {
      if (i < rows_size) {
        unassociated_rows->push_back(i);
      }
      if (j < cols_size) {
        unassociated_cols->push_back(j);
      }
    }
  }
}

}  // namespace

// 匈牙利算法实现，寻找最大权重匹配
void Hungarian(int n, const double* edge, int* perm, double* lx, double* ly) {
  //   CHECK_GT(n, 0);
  //   CHECK(edge);
  //   CHECK(perm);
  //   CHECK(lx);
  //   CHECK(ly);

  // 查找最初的可行性标定: lx[i] + ly[j] >= edge[i][j] for all (i, j).
  for (int i = 0; i < n; ++i) {
    lx[i] = 0;
    ly[i] = 0;
    for (int j = 0; j < n; ++j) {
      lx[i] = std::max(lx[i], edge[i * n + j]);
    }
  }

  // 存储完整匹配结果，存储了排列和逆排列.
  Matching M(n);
  // 使用下列变量构建交替树：其根是一个不匹配的顶点，并且树边在匹配和不匹配之间交替。
  std::vector<int> S;// 树中 'i' 顶点集合，S[0] 是树的根节点.
  OrderedSet T(n);// 树中 'j' 顶点集合
  std::vector<int> parentT;  // T 中元素的父节点在 S 中的索引

  // 在由1x和1y定义的等式子图中，从 S 可达的顶点集合
  OrderedSet N(n);
  std::vector<int> parentN;  // N 中元素的父节点在 S 中的索引

  while (true) {
    // 找到未匹配的 i 顶点
    int i = M.FindUnmatchedI();

     // 如果没有未匹配的 i，匹配完成
    if (i == n) {
      M.CopyTo(perm);
      return;
    }

    // 清空交替树
    S.clear();
    T.clear();
    parentT.clear();

    // 树的根节点为 i
    S.push_back(i);
    FindNeighbours(S, n, edge, lx, ly, &N, &parentN);

    // 如果 N 与 T 完全重合，需要更新标定
    while (true) {
      // 从 N 中选择一个不在 T 中的 j
      if (N == T) {
        ImproveFeasibleLabeling(n, edge, S, T, lx, ly);
        FindNeighbours(S, n, edge, lx, ly, &N, &parentN);
      }

      // 获取与 j 匹配的 i
      int jj = PickY(N, T);
      int j = N[jj];
      T.insert(j);
      parentT.push_back(parentN[jj]);

      // 如果j为free, 有一个交替路径在S[0]和j之间, 所以我们可以增加匹配。如果不行，我们就扩展交替树。
      int z = M.GetI(j);
      if (z == -1) {
        AugmentMatching(S, T, parentT, &M);// 扩展匹配
        break;
      } else {
        S.push_back(z);
        AddNeighbours(z, S.size() - 1, n, edge, lx, ly, &N, &parentN);
      }
    }
  }
}

// 计算最大相似度匹配
void HungarianMaximize(const std::vector<std::vector<double>>& similarities,
                       const double min_similarity,
                       std::vector<std::pair<uint32_t, uint32_t>>* associations,
                       std::vector<uint32_t>* unassociated_rows,
                       std::vector<uint32_t>* unassociated_cols) {
  const uint32_t rows_size = similarities.size();
  const uint32_t cols_size = similarities[0].size();
  const uint32_t num_vertices = std::max(rows_size, cols_size);
  std::vector<double> edge(num_vertices * num_vertices, 0.0);
  std::vector<int> matching(num_vertices, 0);
  for (uint32_t i = 0; i < rows_size; ++i) {
    for (uint32_t j = 0; j < cols_size; ++j) {
      if (similarities[i][j] > min_similarity) {
        edge[i * num_vertices + j] = similarities[i][j];
      }
    }
  }

  std::vector<double> lx(num_vertices);
  std::vector<double> ly(num_vertices);
  Hungarian(num_vertices, edge.data(), matching.data(), &lx[0], &ly[0]);

  associations->clear();
  unassociated_rows->clear();
  unassociated_cols->clear();
  for (uint32_t i = 0; i < num_vertices; ++i) {
    const uint32_t j = matching[i];
    if (edge[i * num_vertices + j] > 0) {
      associations->emplace_back(i, j);
    } else {
      if (i < rows_size) {
        unassociated_rows->push_back(i);
      }
      if (j < cols_size) {
        unassociated_cols->push_back(j);
      }
    }
  }
}
// 计算最小距离匹配
void HungarianMinimize(const std::vector<std::vector<double>>& distance,
                       const double max_distance,
                       std::vector<std::pair<uint32_t, uint32_t>>* associations,
                       std::vector<uint32_t>* unassociated_rows,
                       std::vector<uint32_t>* unassociated_cols) {
  const uint32_t rows_size = distance.size();
  const uint32_t cols_size = distance[0].size();
  const uint32_t num_vertices = std::max(rows_size, cols_size);
  std::vector<double> edge(num_vertices * num_vertices, 0.0);
  std::vector<int> matching(num_vertices, 0);
  for (uint32_t i = 0; i < rows_size; ++i) {
    for (uint32_t j = 0; j < cols_size; ++j) {
      if (distance[i][j] < max_distance) {
        edge[i * num_vertices + j] = max_distance - distance[i][j] + 1.0;
      }
    }
  }

  std::vector<double> lx(num_vertices);
  std::vector<double> ly(num_vertices);
  Hungarian(num_vertices, edge.data(), matching.data(), &lx[0], &ly[0]);

  associations->clear();
  unassociated_rows->clear();
  unassociated_cols->clear();
  for (uint32_t i = 0; i < num_vertices; ++i) {
    const uint32_t j = matching[i];
    if (edge[i * num_vertices + j] > 0) {
      associations->emplace_back(i, j);
    } else {
      if (i < rows_size) {
        unassociated_rows->push_back(i);
      }
      if (j < cols_size) {
        unassociated_cols->push_back(j);
      }
    }
  }
}