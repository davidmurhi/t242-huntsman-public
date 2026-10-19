#!/usr/bin/env bash
# Run pytest and surface each failure as a GitHub annotation, so failures
# show on the pull request without opening the log.
set -o pipefail
python -m pytest "$@" -rfE --tb=line 2>&1 | tee pytest.log
status=$?
grep -E "^(FAILED|ERROR) " pytest.log | head -10 | while IFS= read -r line; do
  echo "::error title=pytest::${line}"
done
grep -E "^/.*\.py:[0-9]+: " pytest.log | head -10 | while IFS= read -r line; do
  echo "::error title=assert::${line}"
done
grep -E "^E  " pytest.log | head -8 | while IFS= read -r line; do
  echo "::error title=detail::${line}"
done
exit $status
