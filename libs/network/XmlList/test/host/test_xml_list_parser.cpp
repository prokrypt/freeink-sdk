#include <XmlListParser.h>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
static const char* kDoc =
  "<?xml version=\"1.0\"?><d:multistatus xmlns:d=\"DAV:\">"
  "<d:response><d:href>/books/</d:href><d:propstat><d:prop><d:resourcetype><d:collection/></d:resourcetype></d:prop></d:propstat></d:response>"
  "<d:response><d:href> /books/a%20b.epub </d:href><d:propstat><d:prop><d:displayname>A B</d:displayname><d:resourcetype/></d:prop></d:propstat></d:response>"
  "</d:multistatus>";
struct Src { const char* p; size_t left; };
int main() {
  std::string url = "href", title = "displayname", none;
  const std::string* const sel[] = {&url, &title, &none, &none};
  std::vector<XmlListParser::RawItem> rows;
  auto* p = new XmlListParser("response", "collection", sel,
      [](void* c, XmlListParser::RawItem& r) { static_cast<std::vector<XmlListParser::RawItem>*>(c)->push_back(r); }, &rows);
  Src s{kDoc, strlen(kDoc)};
  // 7-byte reads exercise element/text boundaries split across chunks.
  bool ok = p->parse([](void* c, char* b, size_t n) { auto* s = static_cast<Src*>(c); size_t k = s->left < 7 ? s->left : 7; if (k > n) k = n; memcpy(b, s->p, k); s->p += k; s->left -= k; return (int)k; }, &s);
  assert(ok && rows.size() == 2);
  assert(rows[0].isDir && rows[0].field[XmlListParser::F_URL] == "/books/");
  assert(!rows[1].isDir && rows[1].field[XmlListParser::F_URL] == "/books/a%20b.epub" && rows[1].field[XmlListParser::F_TITLE] == "A B");
  Src bad{"<a><b></a>", 10};
  XmlListParser q("b", "", sel, [](void*, XmlListParser::RawItem&) {}, nullptr);
  assert(!q.parse([](void* c, char* b, size_t n) { auto* s = static_cast<Src*>(c); size_t k = s->left < n ? s->left : n; memcpy(b, s->p, k); s->p += k; s->left -= k; return (int)k; }, &bad));
  delete p;
  puts("xmllist ok");
}
