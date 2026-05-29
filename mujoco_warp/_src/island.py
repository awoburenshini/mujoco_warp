# Copyright 2026 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import warp as wp

from mujoco_warp._src import types
from mujoco_warp._src.types import ConstraintType
from mujoco_warp._src.types import EqType
from mujoco_warp._src.types import ObjType
from mujoco_warp._src.warp_util import event_scope


@wp.kernel
def _tree_edges(
  # Model:
  nv: int,
  body_treeid: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_treeid: wp.array[int],
  geom_bodyid: wp.array[int],
  site_bodyid: wp.array[int],
  eq_type: wp.array[int],
  eq_obj1id: wp.array[int],
  eq_obj2id: wp.array[int],
  eq_objtype: wp.array[int],
  # Data in:
  nefc_in: wp.array[int],
  contact_geom_in: wp.array[wp.vec2i],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_J_in: wp.array3d[float],
  njmax_in: int,
  # Out:
  tree_tree: wp.array3d[int],  # kernel_analyzer: off
):
  """Find tree edges."""
  # <md>
  # $$G=(V,E),\quad V=\{\,\text{trees }t\,\},\quad (t_a,t_b)\in E \iff \exists\, c:\, t_a,t_b\in\text{trees}(c)\qquad\text{(constraints couple DOF-trees)}$$
  # </md>
  worldid, efcid = wp.tid()

  # skip if beyond active constraints
  if efcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  efc_type = efc_type_in[worldid, efcid]
  efc_id = efc_id_in[worldid, efcid]

  tree0 = int(-1)
  tree1 = int(-1)
  use_generic = int(0)

  # <md>
  # $$c\;\longmapsto\;\bigl(t_0,t_1\bigr)=\bigl(\texttt{tree}(\text{obj}_1),\,\texttt{tree}(\text{obj}_2)\bigr)\qquad\text{(each constraint }c\text{ yields the endpoint tree(s) it links)}$$
  # </md>
  # equality (connect/weld)
  if efc_type == ConstraintType.EQUALITY:
    eq_t = eq_type[efc_id]

    if eq_t == EqType.CONNECT or eq_t == EqType.WELD:
      b1 = eq_obj1id[efc_id]
      b2 = eq_obj2id[efc_id]

      # site semantics
      if eq_objtype[efc_id] == ObjType.SITE:
        b1 = site_bodyid[b1]
        b2 = site_bodyid[b2]

      tree0 = body_treeid[b1]
      tree1 = body_treeid[b2]
    else:
      # JOINT, TENDON, FLEX
      use_generic = 1

  # joint friction
  elif efc_type == ConstraintType.FRICTION_DOF:
    tree0 = dof_treeid[efc_id]

  # joint limit
  elif efc_type == ConstraintType.LIMIT_JOINT:
    tree0 = dof_treeid[jnt_dofadr[efc_id]]

  # contact
  elif (
    efc_type == ConstraintType.CONTACT_FRICTIONLESS
    or efc_type == ConstraintType.CONTACT_PYRAMIDAL
    or efc_type == ConstraintType.CONTACT_ELLIPTIC
  ):
    geom_pair = contact_geom_in[efc_id]
    g1 = geom_pair[0]
    g2 = geom_pair[1]

    # flex contacts have negative geom ids
    if g1 >= 0 and g2 >= 0:
      tree0 = body_treeid[geom_bodyid[g1]]
      tree1 = body_treeid[geom_bodyid[g2]]
    else:
      use_generic = 1

  # generic
  else:
    use_generic = 1

  # handle static bodies
  if use_generic == 0:
    # swap so tree0 is non-negative if possible
    if tree0 < 0 and tree1 >= 0:
      tree0 = tree1
      tree1 = -1

    # <md>
    # $$t_0=t_1\;\Rightarrow\;(t_0,t_0)\in E\ \text{(self-loop)};\qquad t_0\neq t_1\;\Rightarrow\;(t_0,t_1),(t_1,t_0)\in E\quad\text{(symmetric adjacency)}$$
    # </md>
    # mark the edge
    if tree0 >= 0:
      if tree1 < 0 or tree0 == tree1:
        # self-edge
        wp.atomic_max(tree_tree, worldid, tree0, tree0, 1)
      else:
        # cross-tree edge
        t1 = wp.min(tree0, tree1)
        t2 = wp.max(tree0, tree1)
        wp.atomic_max(tree_tree, worldid, t1, t2, 1)
        wp.atomic_max(tree_tree, worldid, t2, t1, 1)
    return

  # <md>
  # $$\text{trees}(c)=\bigl\{\,\texttt{tree}(d)\;:\;J_{c,d}\neq 0\,\bigr\},\qquad (t_a,t_b)\in E\ \ \forall\, t_a,t_b\in\text{trees}(c)\quad\text{(generic: nonzero Jacobian row entries couple their trees)}$$
  # </md>
  # generic: scan Jacobian row
  first_tree = int(-1)
  has_cross_edge = int(0)

  for dof in range(nv):
    # TODO(team): sparse efc_J
    # TODO(team): tree dof skip
    J_val = efc_J_in[worldid, efcid, dof]
    if J_val != 0.0:
      tree = dof_treeid[dof]
      if tree < 0:
        continue
      if first_tree == -1:
        first_tree = tree
      elif tree != first_tree:
        t1 = wp.min(first_tree, tree)
        t2 = wp.max(first_tree, tree)
        wp.atomic_max(tree_tree, worldid, t1, t2, 1)
        has_cross_edge = 1

  if first_tree >= 0 and has_cross_edge == 0:
    wp.atomic_max(tree_tree, worldid, first_tree, first_tree, 1)


def tree_edges(m: types.Model, d: types.Data, tree_tree: wp.array3d[int]):
  """Compute tree-tree adjacency matrix."""
  tree_tree.zero_()
  wp.launch(
    kernel=_tree_edges,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.nv,
      m.body_treeid,
      m.jnt_dofadr,
      m.dof_treeid,
      m.geom_bodyid,
      m.site_bodyid,
      m.eq_type,
      m.eq_obj1id,
      m.eq_obj2id,
      m.eq_objtype,
      d.nefc,
      d.contact.geom,
      d.efc.type,
      d.efc.id,
      d.efc.J,
      d.njmax,
    ],
    outputs=[tree_tree],
  )


@wp.kernel
def _flood_fill(
  # Model:
  ntree: int,
  # In:
  tree_tree_in: wp.array3d[int],
  labels_in: wp.array2d[int],
  stack_in: wp.array2d[int],
  # Data out:
  nisland_out: wp.array[int],
  tree_island_out: wp.array2d[int],
  # Out:
  stack_out: wp.array2d[int],
):
  """DFS flood fill to discover islands using tree_tree matrix."""
  # <md>
  # $$\mathcal{P}_1 \sqcup \mathcal{P}_2 \sqcup \dots \sqcup \mathcal{P}_{n_{\text{island}}} = \{\,t\in V : \deg(t)>0\,\}\qquad\text{(connected components of }G\text{; isolated trees excluded)}$$
  # </md>
  worldid = wp.tid()
  nisland = int(0)

  # iterate over trees
  for i in range(ntree):
    # already assigned
    if labels_in[worldid, i] != -1:
      continue

    # check if tree has any edges
    has_edge = int(0)
    for j in range(ntree):
      if tree_tree_in[worldid, i, j] != 0:
        has_edge = 1
        break
    if has_edge == 0:
      continue

    # DFS: push i onto stack
    nstack = int(0)
    stack_out[worldid, nstack] = i
    nstack = nstack + 1

    while nstack > 0:
      # pop v from stack
      nstack = nstack - 1
      v = stack_in[worldid, nstack]

      # already assigned
      if labels_in[worldid, v] != -1:
        continue

      # <md>
      # $$\text{find}(v)\;\equiv\;k,\qquad \mathcal{P}_k \leftarrow \mathcal{P}_k \cup \{v\}\quad\text{(union: label }v\text{ with the seed's component }k\text{ via DFS closure)}$$
      # </md>
      # assign to current island
      tree_island_out[worldid, v] = nisland

      # push neighbors
      for neighbor in range(ntree):
        if tree_tree_in[worldid, v, neighbor] != 0:
          if labels_in[worldid, neighbor] == -1:
            stack_out[worldid, nstack] = neighbor
            nstack = nstack + 1

    # <md>
    # $$k \leftarrow k+1\qquad\text{(component }\mathcal{P}_k\text{ exhausted; advance to next unlabeled seed)}$$
    # </md>
    # island filled
    nisland = nisland + 1

  nisland_out[worldid] = nisland


@event_scope
def island(m: types.Model, d: types.Data):
  """Discover constraint islands."""
  if m.ntree == 0:
    d.nisland.zero_()
    return

  # <md>
  # $$A \in \{0,1\}^{n_{\text{tree}}\times n_{\text{tree}}},\qquad A_{ab}=1 \iff (t_a,t_b)\in E\quad\text{(symmetric adjacency matrix of }G\text{)}$$
  # </md>
  # Step 1: Find tree edges
  tree_tree = wp.zeros((d.nworld, m.ntree, m.ntree), dtype=int)
  tree_edges(m, d, tree_tree)

  # <md>
  # $$\{\mathcal{P}_k\}_{k=1}^{n_{\text{island}}} = \text{ConnectedComponents}(G),\qquad V=\bigsqcup_k \mathcal{P}_k\ \sqcup\ \{\text{isolated}\}\quad\text{(partition DOF-trees into independent islands)}$$
  # </md>
  # Step 2: DFS flood fill
  d.tree_island.fill_(-1)
  stack_scratch = wp.empty((d.nworld, m.ntree * m.ntree), dtype=int)

  wp.launch(
    _flood_fill,
    dim=d.nworld,
    inputs=[m.ntree, tree_tree, d.tree_island, stack_scratch],
    outputs=[d.nisland, d.tree_island, stack_scratch],
  )
