# Web console (nginx + static app). Built from the repository root
# (compose: context: ..): the COPY sources below are relative to that root
# (ui/nginx.conf, ui/entrypoint.sh, ui/static/).
FROM nginx:1.27-alpine

# poppler-utils: converts table_FINALE_1.0.pdf to table.png at startup
RUN apk add --no-cache poppler-utils

COPY ui/nginx.conf /etc/nginx/conf.d/default.conf
COPY ui/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY ui/static/ /usr/share/nginx/html/

EXPOSE 80
ENTRYPOINT ["/entrypoint.sh"]
