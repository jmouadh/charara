#!/bin/sh
# Convert the table artwork once, then hand over to nginx.
set -e

FIELD=/usr/share/nginx/html/field

if [ -f "$FIELD/table_FINALE_1.0.pdf" ] && [ ! -f "$FIELD/table.png" ]; then
    echo "[ui] converting table_FINALE_1.0.pdf -> table.png (150 dpi)"
    pdftoppm -png -r 150 -singlefile "$FIELD/table_FINALE_1.0.pdf" "$FIELD/table" \
        || echo "[ui] conversion failed - the built-in schematic will be used"
fi

exec nginx -g "daemon off;"
