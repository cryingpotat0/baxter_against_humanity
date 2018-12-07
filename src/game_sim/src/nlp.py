import nltk
from nltk.corpus import stopwords
from gensim.models import Word2Vec
from nltk.corpus import treebank #brown, movie_reviews, treebank
import numpy as np
from nltk.tokenize import word_tokenize 

model = Word2Vec(treebank.sents())

def sentence_to_vec(s):
    stop_words = set(stopwords.words('english')) 
    word_tokens = word_tokenize(s) 
    filtered_sentence = [w for w in word_tokens if not w in stop_words] 
    filtered_sentence = [] 
    for w in word_tokens: 
        if w not in stop_words: 
            filtered_sentence.append(w) 

    avg_vec = np.zeros(len(model['a']))
    for w in filtered_sentence:
        try:
            avg_vec += model[w]
        except KeyError:
            pass
    return avg_vec / len(filtered_sentence)
    


def sentence_similarity(sen1, sen2):
    sen1_vec = sentence_to_vec(sen1)
    sen2_vec = sentence_to_vec(sen2)
    print(sen1_vec, sen2_vec)
    try:
        cosine_similarity = np.dot(sen1_vec, sen2_vec)/(np.linalg.norm(sen1_vec)* np.linalg.norm(sen2_vec))
        return cosine_similarity
    except Exception:
        return 0

