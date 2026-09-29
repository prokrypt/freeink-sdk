#include "XmlListParser.h"

#include <algorithm>
#include <cstring>

namespace {
void splitSelector(const std::string& s, bool& onItemTag, std::string& elem, std::string& attr) {
  if (s.empty()) return;
  if (s[0] == '@') {
    onItemTag = true;
    attr = s.substr(1);
    return;
  }
  const size_t at = s.find('@');
  elem = at == std::string::npos ? s : s.substr(0, at);
  if (at != std::string::npos) attr = s.substr(at + 1);
}

const char* localName(const XML_Char* name) {
  const char* colon = strrchr(name, ':');
  return colon ? colon + 1 : name;
}

const char* findAttr(const XML_Char** atts, const std::string& attr) {
  for (int i = 0; atts[i]; i += 2) {
    if (attr == atts[i]) return atts[i + 1];
  }
  return nullptr;
}

void trim(std::string& s) {
  const size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    s.clear();
    return;
  }
  const size_t e = s.find_last_not_of(" \t\r\n");
  s = s.substr(b, e - b + 1);
}
}  // namespace

XmlListParser::XmlListParser(const std::string& itemName, const std::string& containerName,
                             const std::string* const (&selectors)[F_COUNT], const ItemSink sink, void* sinkCtx)
    : item(itemName), container(containerName), sink(sink), sinkCtx(sinkCtx) {
  for (int i = 0; i < F_COUNT; i++) splitSelector(*selectors[i], sel[i].onItemTag, sel[i].elem, sel[i].attr);
}

bool XmlListParser::parse(const ReadFn read, void* readCtx) {
  XML_Parser p = XML_ParserCreate(nullptr);
  if (!p) return false;
  XML_SetUserData(p, this);
  XML_SetElementHandler(
      p,
      [](void* self, const XML_Char* name, const XML_Char** atts) {
        static_cast<XmlListParser*>(self)->onStart(name, atts);
      },
      [](void* self, const XML_Char* name) { static_cast<XmlListParser*>(self)->onEnd(name); });
  XML_SetCharacterDataHandler(
      p, [](void* self, const XML_Char* s, int len) { static_cast<XmlListParser*>(self)->onText(s, len); });
  bool ok = true;
  for (;;) {
    const int n = read(readCtx, buf, sizeof(buf));
    const bool last = n <= 0;
    if (XML_Parse(p, buf, last ? 0 : n, last ? XML_TRUE : XML_FALSE) != XML_STATUS_OK) {
      ok = false;
      break;
    }
    if (last) break;
  }
  XML_ParserFree(p);
  return ok;
}

void XmlListParser::onStart(const XML_Char* name, const XML_Char** atts) {
  depth++;
  const char* local = localName(name);
  if (itemDepth < 0) {
    if (parsedItems < MAX_ITEMS && item == local) {
      itemDepth = depth;
      current = RawItem{};
      capturingMask = 0;
      for (int i = 0; i < F_COUNT; i++) {
        done[i] = !sel[i].isSet();
        if (sel[i].onItemTag) {
          const char* v = findAttr(atts, sel[i].attr);
          if (v) current.field[i] = v;
          done[i] = true;
        }
      }
    }
    return;
  }
  // Inside an item: a child element ends any leading-text capture.
  capturingMask = 0;
  if (!container.empty() && container == local) current.isDir = true;
  for (int i = 0; i < F_COUNT; i++) {
    if (done[i] || sel[i].onItemTag || sel[i].elem != local) continue;
    done[i] = true;  // first matching descendant wins
    if (!sel[i].attr.empty()) {
      const char* v = findAttr(atts, sel[i].attr);
      if (v) current.field[i] = v;
    } else {
      capturingMask |= 1u << i;
    }
  }
}

void XmlListParser::onEnd(const XML_Char* name) {
  if (itemDepth >= 0) {
    capturingMask = 0;
    if (depth == itemDepth && item == localName(name)) {
      for (auto& f : current.field) trim(f);
      ++parsedItems;
      sink(sinkCtx, current);
      itemDepth = -1;
    }
  }
  depth--;
}

void XmlListParser::onText(const XML_Char* s, const int len) {
  if (itemDepth < 0 || capturingMask == 0) return;
  for (int i = 0; i < F_COUNT; i++) {
    if (!(capturingMask & (1u << i)) || current.field[i].size() >= MAX_FIELD_CHARS) continue;
    current.field[i].append(s, std::min<size_t>(len, MAX_FIELD_CHARS - current.field[i].size()));
  }
}
