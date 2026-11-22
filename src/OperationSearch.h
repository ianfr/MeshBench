#pragma once

// Tiny helper for the Ctrl+Space operation launcher: a query is split into
// whitespace-separated terms and every term must appear in the action text.
// Kept free of widgets so it can be unit-tested.

#include <QString>
#include <QStringList>

namespace meshbench {

inline bool operationSearchMatches(const QString& text, const QString& query) {
  const QStringList terms =
      query.split(QLatin1Char(' '), Qt::SkipEmptyParts);
  for (const QString& term : terms) {
    if (!text.contains(term, Qt::CaseInsensitive))
      return false;
  }
  return true;
}

} // namespace meshbench
