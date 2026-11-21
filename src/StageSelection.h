#pragma once

// Helpers for the explicit right-click stage selection used before
// Combine. Qt's tree selection can be cleared by the right-click that
// opens the context menu, so the selected stages are also kept in a
// MainWindow-side set and merged back in when the menu action runs.

#include <QSet>

namespace meshbench {

inline QSet<int> toggleStageSelection(QSet<int> selection, int stage,
                                      bool selected) {
  if (stage < 0)
    return selection;
  if (selected)
    selection.insert(stage);
  else
    selection.remove(stage);
  return selection;
}

inline QSet<int> mergeStageSelections(const QSet<int>& a,
                                      const QSet<int>& b) {
  QSet<int> merged = a;
  merged.unite(b);
  return merged;
}

// Explicit Combine picks win over Qt's transient tree selection. Fall
// back to the tree only when no explicit picks exist.
inline QSet<int> resolveStageSelection(const QSet<int>& explicitSelection,
                                       const QSet<int>& treeSelection) {
  return explicitSelection.isEmpty() ? treeSelection : explicitSelection;
}

inline QSet<int> validStageSelection(const QSet<int>& selection,
                                     int nodeCount) {
  QSet<int> valid;
  for (int stage : selection) {
    if (stage >= 0 && stage < nodeCount)
      valid.insert(stage);
  }
  return valid;
}

} // namespace meshbench
