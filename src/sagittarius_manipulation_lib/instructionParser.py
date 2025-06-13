import re
import jieba.posseg as psg


class InstructionParser:
    def __init__(self):
        # 动作映射表
        self.action_map = {
            '拿': 'pick',
            '取': 'pick',
            '捡': 'pick',
            '放到': 'place',
            '放置': 'place',
            '摆': 'place',
            '移动': 'move',
            '移到': 'move',
            '搬到': 'move',
        }
        # 新增结构化动作类型映射
        self.structured_action_map = {
            'pick': 'take_object',
            'place': 'put_object',
            'move': 'move_object'
        }
        self.prepositions = {'在', '到', '至', '于', '向'}
        self.location_keywords = {'上', '下', '左', '右', '里', '内', '旁'}

    def clean_text(self, text):
        #文本预处理
        text = re.sub(r'[^\w\s]', '', text)
        text = re.sub(r'\s+', ' ', text).strip()
        return text

    def parse(self, text):
        #主解析函数
        cleaned = self.clean_text(text)
        words = list(psg.cut(cleaned))

        intent = self._get_intent(words)
        locations = self._find_locations(words)  # 先提取位置
        objects = self._find_objects(words, locations)
        clarification = self._check_ambiguity(intent, objects, locations)

        # 结构化输出
        return {
            'NLU结果': self._build_nlu_result(intent, objects, locations),
            '需要澄清': clarification
        }

    def _build_nlu_result(self, intent, objects, locations):
        #构建NLU结果结构
        entities = []
        # 添加对象实体
        for obj in objects:
            entities.append({
                'word': obj,
                'type': 'object',
                'custom': True
            })
        # 添加位置实体
        for loc in locations:
            entities.append({
                'word': loc,
                'type': 'location',
                'custom': True
            })

        return {
            'intent': self.structured_action_map.get(intent, 'unknown'),
            'action': intent,
            'object': objects[0] if objects else None,
            'location': max(locations, key=len) if locations else None,
            'entities': entities
        }

    def _get_intent(self, words):
        #意图提取
        for word, flag in words:
            if flag.startswith('v') and word in self.action_map:
                return self.action_map[word]
        return None

    def _find_objects(self, words, locations):
        #提取物体（排除位置中的名词）
        objects = []
        location_words = self._get_location_words(locations)

        # 处理"把"字结构
        ba_index = next((i for i, (w, f) in enumerate(words) if w == '把'), -1)
        if ba_index != -1:
            for w, f in words[ba_index + 1:]:
                if f.startswith('n') and w not in location_words:
                    objects.append(w)
                elif f in {'c', 'uj'}:
                    continue
                else:
                    break

        # 处理动词后介词前宾语
        '''
        verb_indices = [i for i, (w, f) in enumerate(words) if f.startswith('v')]
        for vi in verb_indices:
            obj = []
            for w, f in words[vi + 1:]:
                # 遇到介词或方位词停止收集
                if w in self.prepositions or f == 'f':
                    break
                if f.startswith('n') and w not in location_words:
                    obj.append(w)
                elif w in {'和', '与','以及'}:
                    continue
                else:
                    break
            objects.extend(obj)
        '''
        return list(set(objects))


    def _find_locations(self, words: list) -> list:
        #位置提取（从动词之后开始搜索）
        locations = []

        #动词位置
        verb_index = -1
        for i, (word, flag) in enumerate(words):
            if flag.startswith('v'):
                verb_index = i
                break

        if verb_index == -1:
            return locations

        #从动词之后开始搜索位置
        post_verb_words = words[verb_index + 1:]

        # 处理介词结构
        for i, (w, f) in enumerate(post_verb_words):
            if w in self.prepositions or f == 'p':  # 介词标记
                loc = []
                for w2, f2 in post_verb_words[i + 1:]:
                    # 允许名词+方位词的复合结构
                    if (f2 in {'n', 's', 'f', 'l'} or
                            w2 in self.location_keywords or
                            w2 in self.custom_locations):
                        loc.append(w2)
                    else:
                        break
                if loc:
                    locations.append(''.join(loc))

            # 直接方位词（如"上面"）
            elif f == 'f' or w in self.location_keywords:
                locations.append(w)

        #处理直接跟在动词后的方位词（如"放到桌上"）
        direct_loc = []
        for w, f in post_verb_words:
            if (f == 'f' or w in self.location_keywords) and not direct_loc:
                direct_loc.append(w)
            elif direct_loc and (f not in {'n', 's', 'f', 'l'}):
                break
        if direct_loc:
            locations.insert(0, ''.join(direct_loc))

        return list(set(locations))

    def _get_location_words(self, locations):
        #获取位置中的独立词汇
        words = []
        for loc in locations:
            # 简单分词逻辑
            if len(loc) > 2:
                words.extend([loc[:2], loc[2:]])
            else:
                words.append(loc)
        return set(words)


    def _is_location(self, word, flag):
        return flag in {'s', 'f', 'l'} or word in {'上', '下', '左', '右'}

    def _check_ambiguity(self, intent, objects, locations):
        needs = []
        if not intent:
            return "请明确您的操作意图（如拿取、放置、移动）"
        if not objects:
            needs.append("对象")
        if intent in ['place', 'move'] and not locations:
            needs.append("位置")
        return f"请明确：{','.join(needs)}" if needs else None


