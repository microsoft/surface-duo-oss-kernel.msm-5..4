#!/bin/bash
MODULE_LIST="$1"
MODULE_UNSIGNED_BIN="$2"
retval=0

echo "Adding $MODULE_UNSIGNED_BIN to $MODULE_LIST."
echo $MODULE_UNSIGNED_BIN >> $MODULE_LIST

return $retval &>/dev/null || exit $retval