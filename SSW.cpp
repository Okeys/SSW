#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QRegExp>
#include <QTextStream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

enum class TokenKind
{
    FOR,
    WHILE,
    IF,
    ELSE,
    INCR,
    DECR,
    L_RBR,
    R_RBR,
    L_BR,
    R_BR,
    SMCLN,
    OP,
    LET,
    VAR,
    ASSIGN_OP,
    COMMA,
    NOT_EQUAL,
    IN,
    SPACE,
    NUMBER,
    NEW,
    END_OF_FILE,
    STR,
    COMMENT
};

#define __OUT_ENUM(kind)                                                  \
    case TokenKind::kind: {                                               \
        return out << "" << #kind;                                        \
    }
std::ostream& operator<<(std::ostream& out, const TokenKind kind) {
    switch (kind) {
        __OUT_ENUM(FOR);
        __OUT_ENUM(STR);
        __OUT_ENUM(COMMENT);
        __OUT_ENUM(WHILE);
        __OUT_ENUM(IF);
        __OUT_ENUM(ELSE);
        __OUT_ENUM(INCR);
        __OUT_ENUM(DECR);
        __OUT_ENUM(L_RBR);
        __OUT_ENUM(R_RBR);
        __OUT_ENUM(L_BR);
        __OUT_ENUM(R_BR);
        __OUT_ENUM(SMCLN);
        __OUT_ENUM(OP);
        __OUT_ENUM(LET);
        __OUT_ENUM(VAR);
        __OUT_ENUM(ASSIGN_OP);
        __OUT_ENUM(COMMA);
        __OUT_ENUM(NOT_EQUAL);
        __OUT_ENUM(IN);
        __OUT_ENUM(SPACE);
        __OUT_ENUM(NUMBER);
        __OUT_ENUM(NEW);
        __OUT_ENUM(END_OF_FILE);
    }
}

struct Lexem {
    TokenKind kind;
    QRegExp   reg;
    int       priority;
    bool      skipable;
};


struct Token {
    TokenKind kind;
    QString   value;
};

struct MatchResult {
    Lexem   pattern;
    QString match;
};

std::vector<Token> lexer(QString S, bool printLex) {
    int startpos = 0;
    std::vector<Token> tokens;

    QRegExp strPatt("\".*\"");
    strPatt.setMinimal(true);
    QRegExp commPatt("/\\*.*\\*/");
    commPatt.setMinimal(true);

    std::vector<Lexem> Lexems{
        {TokenKind::FOR, QRegExp("for"), 1, false},
        {TokenKind::WHILE, QRegExp("while"), 1, false},
        {TokenKind::IF, QRegExp("if"), 1, false},
        {TokenKind::ELSE, QRegExp("else"), 1, false},
        {TokenKind::INCR, QRegExp("\\+\\+"), 0, false},
        {TokenKind::DECR, QRegExp("\\-\\-"), 0, false},
        {TokenKind::L_RBR, QRegExp("\\("), 0, false},
        {TokenKind::R_RBR, QRegExp("\\)"), 0, false},
        {TokenKind::L_BR, QRegExp("\\{"), 0, false},
        {TokenKind::R_BR, QRegExp("\\}"), 0, false},
        {TokenKind::SMCLN, QRegExp("\\;"), 0, false},
        {TokenKind::OP, QRegExp("[+-/*<>|&!]+"), 0, false},
        {TokenKind::OP, QRegExp("=="), 3, false},
        {TokenKind::LET, QRegExp("let"), 0, false},
        {TokenKind::VAR, QRegExp("[a-zA-Z][a-zA-Z0-9]*"), 0, false},
        {TokenKind::ASSIGN_OP, QRegExp("="), 2, false},
        {TokenKind::COMMA, QRegExp(","), 1, false},
        {TokenKind::NOT_EQUAL, QRegExp("!="), 0, false},
        {TokenKind::IN, QRegExp("in"), 0, false},
        {TokenKind::SPACE, QRegExp("\\s+"), 0, true},
        {TokenKind::NEW, QRegExp("new"), 1, false},
        {TokenKind::NUMBER, QRegExp("[0-9.]+"), 0, false},
        {TokenKind::STR, strPatt, 0, false},
        {TokenKind::COMMENT, commPatt, 0, true}};

    while (startpos < S.length()) {
        std::vector<MatchResult> matches;
        for (const auto& pattern : Lexems) {
            int pos = pattern.reg.indexIn(S, startpos);
            if (pos == startpos) {
                QString found_piece = S.mid(
                    startpos, pattern.reg.matchedLength());
                matches.push_back({pattern, found_piece});
            }
        }

        std::sort(
            matches.begin(),
            matches.end(),
            [](const MatchResult& left, const MatchResult& right) {
                return left.pattern.priority > right.pattern.priority;
            });

        if (matches.size() == 0) {
            std::cerr << "No pattern matching for token at position "
                      << S[startpos].toLatin1() << "\n";
            abort();

        } else {
            const MatchResult& best = matches[0];
            if (best.pattern.skipable) {
                startpos += best.pattern.reg.matchedLength();
            } else {
                tokens.push_back({best.pattern.kind, best.match});
                startpos += best.pattern.reg.matchedLength();
            }
        }
    }

    if (printLex) {
        for (const auto& k : tokens) {
            std::cout << "| " << k.kind << " |"
                      << "    [ " << k.value.toStdString() << " ]"
                      << std::endl;
        }
    }
    return tokens;
}

std::ostream& operator<<(std::ostream& out, const Token match) {
    return out << "[" << match.kind << " " << match.value.toStdString()
               << "]";
}

enum class Ast_Kind
{
    expr,
    value,
    OP,
    VAR,
    NUMBER,
    STR,
    statement,
    assign_statement,
    lang,
    if_check,
    if_branch,
    if_condition,
    if_body,
    else_branch,
    else_head,
    else_body,
    while_statement,
    while_condition,
    while_body,
    new_expr,
    function_call
};

#define __OUT_ENUM_AST(kind)                                              \
    case Ast_Kind::kind: {                                                \
        return out << "" << #kind << ": ";                                \
    }

std::ostream& operator<<(std::ostream& out, const Ast_Kind kind) {
    switch (kind) {
        __OUT_ENUM_AST(expr);
        __OUT_ENUM_AST(value);
        __OUT_ENUM_AST(OP);
        __OUT_ENUM_AST(STR);
        __OUT_ENUM_AST(VAR);
        __OUT_ENUM_AST(NUMBER);
        __OUT_ENUM_AST(statement);
        __OUT_ENUM_AST(assign_statement);
        __OUT_ENUM_AST(lang);
        __OUT_ENUM_AST(if_check);
        __OUT_ENUM_AST(if_branch);
        __OUT_ENUM_AST(if_condition);
        __OUT_ENUM_AST(if_body);
        __OUT_ENUM_AST(else_branch);
        __OUT_ENUM_AST(else_head);
        __OUT_ENUM_AST(else_body);
        __OUT_ENUM_AST(while_statement);
        __OUT_ENUM_AST(while_condition);
        __OUT_ENUM_AST(while_body);
        __OUT_ENUM_AST(new_expr);
        __OUT_ENUM_AST(function_call);
    }
}

class Ast
{
  public:
    Ast_Kind kind;
    QString str;
    std::vector<Ast*> subnodes;
    Ast(Ast_Kind _kind, std::initializer_list<Ast*> _subnodes)
        : kind(_kind), subnodes(_subnodes) {
    }
    Ast(Ast_Kind _kind, QString _value, std::initializer_list<Ast*> _subnodes)
        : kind(_kind), str(_value), subnodes(_subnodes) {
    }
    Ast(Ast_Kind _kind, QString _str) : kind(_kind), str(_str) {
    }
    ~Ast() {
    }

    void print(int level) {
        std::cout << std::string(level * 2, ' ') << this->kind
                  << this->str.toStdString() << "\n";
        for (auto node : subnodes) {
            node->print(level + 1);
        }
    }

    void push_back(Ast* other) {
        this->subnodes.push_back(other);
    }

    Ast* operator[](int idx) {
        return this->subnodes[idx];
    }
};

class Parser
{
    std::vector<Token> tokens;
    Token currentToken(int offset = 0);
    Token match(TokenKind kind);
    Ast*  expr();
    Ast*  value();
    Ast*  statement();
    Ast*  assign_statement();
    Ast*  lang();
    Ast*  if_check();
    Ast*  if_branch();
    Ast*  if_condition();
    Ast*  if_body();
    Ast*  else_branch();
    Ast*  else_head();
    Ast*  else_body();
    Ast*  while_statement();
    Ast*  while_condition();
    Ast*  while_head();
    Ast*  while_body();
    Ast*  new_expr();
    Ast*  function_call();
    bool  token_check(TokenKind kind);
    void  next();

  public:
    int  position = 0;
    Ast* parse();
    Parser(std::vector<Token> _tokens) : tokens(_tokens) {
    }
    Ast* lhs;
    Ast* rhs;
};

void Parser::next() {
    this->position++;
}

Ast* Parser::parse() {
    return lang();
}

bool Parser::token_check(TokenKind kind) {
    return currentToken().kind == kind;
}

Token Parser::currentToken(int offset) {
    if (position + offset >= tokens.size()) {
        return {TokenKind::END_OF_FILE, ""};
    } else {
        return this->tokens[this->position + offset];
    }
}

Token Parser::match(TokenKind kind) {
    if (tokens[this->position].kind == kind) {
        auto res = currentToken();
        this->position++;
        return res;
    } else {
        std::cerr << "Token at position " << position
                  << " does not match: expected " << kind << " but got "
                  << tokens[this->position].kind << std::endl;
        throw "Token doesn't match";
    }
}

Ast* Parser::lang() {
    auto ast = new Ast(Ast_Kind::lang, {});
    ast->push_back(statement());
    while (token_check(TokenKind::LET) || token_check(TokenKind::VAR) || token_check(TokenKind::IF)
           || token_check(TokenKind::WHILE) || token_check(TokenKind::FOR)
           || token_check(TokenKind::NUMBER) || token_check(TokenKind::OP)
           || token_check(TokenKind::L_RBR)) {
        ast->push_back(statement());
    }
    return ast;
}

Ast* Parser::assign_statement() {
    auto ast = new Ast(Ast_Kind::assign_statement, {});
    ast->push_back(new Ast(Ast_Kind::VAR, match(TokenKind::VAR).value));
    match(TokenKind::ASSIGN_OP);
    if (currentToken(+1).kind == TokenKind::L_RBR) {
        ast->push_back(function_call());
    } else {
        ast->push_back(expr());
        match(TokenKind::SMCLN);
    }
    return ast;
}

Ast* Parser::if_check() {
    auto ast = new Ast(Ast_Kind::if_check, {});
    ast->push_back(if_branch());
    if (token_check(TokenKind::ELSE)) {
        ast->push_back(else_branch());
    }
    return ast;
}

Ast* Parser::if_branch() {
    auto ast = new Ast(Ast_Kind::if_branch, {if_condition(), if_body()});
    return ast;
}

Ast* Parser::if_condition() {
    auto ast = new Ast(Ast_Kind::if_condition, {});
    match(TokenKind::IF);
    ast->push_back(expr());
    return ast;
}

Ast* Parser::if_body() {
    auto ast = new Ast(Ast_Kind::if_body, {});
    match(TokenKind::L_BR);
    while (token_check(TokenKind::LET) || token_check(TokenKind::IF) || token_check(TokenKind::WHILE)
           || token_check(TokenKind::FOR) || token_check(TokenKind::NUMBER)
           || token_check(TokenKind::VAR) || token_check(TokenKind::OP)
           || token_check(TokenKind::L_RBR)) {
        ast->push_back(statement());
    }
    match(TokenKind::R_BR);
    return ast;
}

Ast* Parser::else_branch() {
    auto ast = new Ast(Ast_Kind::else_branch, {else_head(), else_body()});
    return ast;
}

Ast* Parser::else_head() {
    auto ast = new Ast(Ast_Kind::else_head, {});
    match(TokenKind::ELSE);
    return ast;
}

Ast* Parser::else_body() {
    auto ast = new Ast(Ast_Kind::else_body, {});
    match(TokenKind::L_BR);
    while (token_check(TokenKind::LET) || token_check(TokenKind::IF) || token_check(TokenKind::WHILE)
           || token_check(TokenKind::FOR) || token_check(TokenKind::NUMBER)
           || token_check(TokenKind::VAR) || token_check(TokenKind::OP)
           || token_check(TokenKind::L_RBR)) {
        ast->push_back(statement());
    }
    match(TokenKind::R_BR);
    return ast;
}

Ast* Parser::while_statement() {
    auto ast = new Ast(
        Ast_Kind::while_statement, {while_condition(), while_body()});
    return ast;
}

Ast* Parser::while_condition() {
    auto ast = new Ast(Ast_Kind::while_condition, {});
    match(TokenKind::WHILE);
    ast->push_back(expr());
    return ast;
}

Ast* Parser::while_body() {
    auto ast = new Ast(Ast_Kind::while_body, {});
    match(TokenKind::L_BR);
    ast->push_back(statement());
    while (token_check(TokenKind::LET) || token_check(TokenKind::IF) || token_check(TokenKind::WHILE)
           || token_check(TokenKind::FOR) || token_check(TokenKind::NUMBER)
           || token_check(TokenKind::VAR) || token_check(TokenKind::OP)
           || token_check(TokenKind::L_RBR)) {
        ast->push_back(statement());
    }
    match(TokenKind::R_BR);
    return ast;
}

Ast* Parser::new_expr() {
    auto ast = new Ast(Ast_Kind::new_expr, {});
    match(TokenKind::NEW);
    match(TokenKind::VAR);
    return ast;
}

Ast* Parser::function_call() {
    // function_call ::= VAR "(" expr? ("," expr)* ");"
    // var_decl ::= "let" VAR = expr
    auto ast = new Ast(Ast_Kind::function_call, currentToken().value, {});
    next();
    match(TokenKind::L_RBR);
    while (token_check(TokenKind::NUMBER) || token_check(TokenKind::VAR)
           || token_check(TokenKind::L_RBR) || token_check(TokenKind::STR)) {
        ast->push_back(expr());
        if (token_check(TokenKind::COMMA)) {
            match(TokenKind::COMMA);
        } else {
            break;
        }
    }
    match(TokenKind::R_RBR);
    match(TokenKind::SMCLN);
    return ast;
}

Ast* Parser::value() {
    auto ast = new Ast(Ast_Kind::value, {});
    switch (currentToken().kind) {
        case TokenKind::VAR:
            ast->push_back(
                new Ast(Ast_Kind::VAR, match(TokenKind::VAR).value));
            break;
        case TokenKind::NUMBER:
            ast->push_back(
                new Ast(Ast_Kind::NUMBER, match(TokenKind::NUMBER).value));
            break;
        default: throw "Error. This isn't a value";
    }
    return ast;
}

Ast* Parser::expr() {
    if (currentToken().kind == TokenKind::NEW) {
        next();
        auto result = new Ast(Ast_Kind::new_expr, currentToken().value);
        next();
        return result;
    }
    std::vector<Token> exprTokens;
    if (currentToken().kind == TokenKind::L_RBR) {
        exprTokens.push_back(currentToken());
        next();
        int balance = 1;
        while (balance > 0) {
            switch (currentToken().kind) {
                case TokenKind::L_RBR: {
                    ++balance;
                    break;
                }
                case TokenKind::R_RBR: {
                    --balance;
                    break;
                }
                default: break;
            }

            exprTokens.push_back(currentToken());
            next();
        }
    } else {
        while (
            !(token_check(TokenKind::SMCLN) || token_check(TokenKind::COMMA)
              || token_check(TokenKind::R_RBR))) {
            exprTokens.push_back(currentToken());
            next();
        }
    }

    std::map<QString, int> prec;
    prec["+"] = 2;
    prec["*"] = 1;
    prec["/"] = 1;
    prec["-"] = 2;
    std::vector<Token> stack;
    std::vector<Token> stack_res;
    std::vector<Ast*>  evalStack;

    for (const auto& token : exprTokens) {
        switch (token.kind) {
            case TokenKind::OP: {
                while (!stack.empty()) {
                    if (prec[token.value] > prec[stack.back().value]) {
                        if (stack.back().kind != TokenKind::L_RBR) {
                            stack_res.push_back(stack.back());
                        }
                        stack.pop_back();
                    } else {
                        break;
                    }
                }
                stack.push_back(token);
                break;
            }
            case TokenKind::L_RBR: {
                stack.push_back(token);
                break;
            }
            case TokenKind::R_RBR: {
                while (!stack.empty()
                       && stack.back().kind != TokenKind::L_RBR) {
                    stack_res.push_back(stack.back());
                    stack.pop_back();
                }
                if (!stack.empty()) {
                    stack.pop_back();
                }
                break;
            }
            default: {
                stack_res.push_back(token);
            }
        }
    }

    while (!stack.empty()) {
        stack_res.push_back(stack.back());
        stack.pop_back();
    }
    for (auto& token : stack_res) {
        if (token.kind == TokenKind::NUMBER) {
            evalStack.push_back(new Ast(Ast_Kind::NUMBER, token.value));
        } else if (token.kind == TokenKind::STR) {
            evalStack.push_back(new Ast(Ast_Kind::STR, token.value));
        } else if (token.kind == TokenKind::VAR) {
            evalStack.push_back(new Ast(Ast_Kind::VAR, token.value));
        } else if (token.kind == TokenKind::OP) {
            Ast* lhs = evalStack.back();
            evalStack.pop_back();
            Ast* rhs = evalStack.back();
            evalStack.pop_back();

            evalStack.push_back(
                new Ast(Ast_Kind::OP, token.value, {lhs, rhs}));
        }
    }
    return evalStack[0];
}

Ast* Parser::statement() {
    auto ast = new Ast(Ast_Kind::statement, {});
    switch (currentToken().kind) {
        case TokenKind::LET:
            ast->push_back(assign_statement());
            break;
        case TokenKind::IF:
            ast->push_back(if_check());
            break;
        case TokenKind::WHILE:
            ast->push_back(while_statement());
            break;
        case TokenKind::NUMBER:
            ast->push_back(expr());
            break;
        case TokenKind::VAR:
            if (currentToken(+1).kind == TokenKind::ASSIGN_OP) {
                ast->push_back(assign_statement());
            } else {
                ast->push_back(function_call());
            }
            break;
        case TokenKind::OP:
            ast->push_back(expr());
            break;
        case TokenKind::L_RBR:
            ast->push_back(expr());
            break;
        default: throw "Error. This isn't a statement";
    }
    return ast;
}

struct Value;

struct Node {
    Value* data;
    Node*  prev;
    Node*  next;

    Node() {
        data = nullptr;
        prev = nullptr;
        next = nullptr;
    }

    void insert(Value* newdata) {
        auto newNode  = new Node();
        newNode->data = newdata;
        newNode->next = nullptr;

        auto ptr = this;
        while (ptr->next != nullptr) {
            ptr = ptr->next;
        }

        ptr->next     = newNode;
        newNode->prev = ptr;
    }

    Value* get(int newdata) {
        auto ptr   = this;
        int  index = 0;
        while (ptr != nullptr) {
            if (index == newdata) {
                return ptr->data;
            } else {
                ptr = ptr->next;
                ++index;
            }
        }
    }
};

bool operator==(const Value& a, const Value& b);

class HashMapTable
{
  public:
    struct Pair {
        Value* key;
        Value* value;
        Pair() = default;
    };

    struct Bucket {
        std::vector<Pair> entries;
        Bucket() = default;
    };

    std::vector<Bucket> bucket;

    HashMapTable() {
        elements_amount = 0;
        bucket = std::vector<Bucket>(4);
    };

    int elements_amount;

    int Hash(Value* key);

    void insert(Value* key, Value* value, bool recount = true);

    Value* get_table(Value* key);

    void resize();
};

enum class ValueKind
{
    Int,
    HashTable,
    LinkedList,
    String,
    Bool,
    Var
};

struct Value {
  private:
    ValueKind kind;
    int intVal;
    QString strVal;
    bool boolVal;
    Node* listVal;
    HashMapTable* tableVal;

  public:
    Value() = default;

    int hash() {
        return this->intVal;
    }

    friend bool operator==(const Value& a, const Value& b);

    int getIntVal() const {
        assert(kind == ValueKind::Int);
        return intVal;
    }

    QString getStrVal() const {
        assert(kind == ValueKind::String);
        return strVal;
    }
    QString getVarVal() const {
        assert(kind == ValueKind::Var);
        return strVal;
    }

    bool getBoolVal() const {
        assert(kind == ValueKind::Bool);
        return boolVal;
    }

    Node* getListVal() const {
        assert(kind == ValueKind::LinkedList);
        return listVal;
    }

    HashMapTable* getTableVal() const {
        assert(kind == ValueKind::HashTable);
        return tableVal;
    }

    ValueKind getKind() const {
        return kind;
    }

    Value(int val) : kind(ValueKind::Int), intVal(val){};
    Value(QString val) : kind(ValueKind::String), strVal(val){};
    Value(ValueKind _kind, QString _string)
        : kind(_kind), strVal(_string){};
    Value(bool val) : kind(ValueKind::Bool), boolVal(val){};
    Value(unsigned long val) : kind(ValueKind::Int), intVal(val){};
    Value(Node* _node) : kind(ValueKind::LinkedList), listVal(_node) {
        std::cout << "Called new value constructor with list argument\n";
        listVal->data = new Value(0);
    }

    Value(HashMapTable* _element)
        : kind(ValueKind::HashTable), tableVal(_element) {
        std::cout << "Called new value constructor with table argument\n";
    }

    Value* newCopy() {
        auto result = new Value();
        *result     = *this;
        return result;
    }
};

void HashMapTable::insert(Value* key, Value* value, bool recount) {
    auto idx = key->hash() % this->bucket.size();
    if (elements_amount * 2 > this->bucket.size()) {
        resize();
    }
    if (bucket[idx].entries.size() == 0) {
        bucket[idx].entries.push_back({key, value});
        if (recount) {
            elements_amount++;
        }
    } else {
        for (size_t pair_idx = 0; pair_idx < this->bucket[idx].entries.size(); ++pair_idx) {
            if (*bucket[idx].entries[pair_idx].key == *key) {
                bucket[idx].entries[pair_idx].value = value;
                return;
            }
        }
        bucket[idx].entries.push_back({key, value});
        if (recount) {
            elements_amount++;
        }
    }
}


void HashMapTable::resize() {
    auto old_bucket = this->bucket;
    bucket = std::vector<Bucket>(this->bucket.size() * 2);
    std::cout << "Resize to " << bucket.size() << " buckets " << std::endl;
    for (auto& old : old_bucket) {
        for (auto& e : old.entries) {
            this->insert(e.key, e.value, false);
        }
    }
}

bool operator==(const Value& a, const Value& b) {
    bool result = a.kind == b.kind;
    if (result) {
        switch (a.kind) {
            case ValueKind::Int: return a.intVal == b.intVal;
            case ValueKind::String: return a.strVal == b.strVal;
        }
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Value& value);

std::ostream& operator<<(std::ostream& os, Node* value) {
    auto ptr = value;
    while (ptr != nullptr) {
        if (ptr->data == nullptr) {
            os << "NO DATA";
        } else {
            os << "->" << *ptr->data;
        }
        ptr = ptr->next;
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const Value& value) {
    os << "[";
    switch (value.getKind()) {
        case ValueKind::Int: {
            os << "Int " << value.getIntVal();
            break;
        }
        case ValueKind::Var: {
            os << "Var " << value.getVarVal().toStdString();
            break;
        }
        case ValueKind::String: {
            os << "String " << value.getStrVal().toStdString();
            break;
        }
        case ValueKind::Bool: {
            os << "Bool ";
            if (value.getBoolVal()) {
                os << "true";
            } else {
                os << "false";
            }
            break;
        }
        case ValueKind::LinkedList: {
            os << "List ";
            os << value.getListVal();
            break;
        }
        case ValueKind::HashTable: {
            os << "Table ";
            int cnt = 0;
            for (const auto& b : value.getTableVal()->bucket) {
                for (const auto& p : b.entries) {
                    if (cnt > 0) {
                        os << ", ";
                    }
                    os << "{" << *p.key << ": " << *p.value << "}";
                    ++cnt;
                }
            }
            break;
        }
    }

    os << "]";

    return os;
}

Value* HashMapTable::get_table(Value* key) {
    auto hash = key->hash();
    auto idx  = hash % this->bucket.size();
    for (size_t pair_idx = 0; pair_idx < this->bucket[idx].entries.size();
         ++pair_idx) {
        if (this->bucket[idx].entries.size() == 0) {
            std::cerr << "The bucket is empty" << std::endl;
            throw "Empty bucket";
        } else if (*this->bucket[idx].entries[pair_idx].key == *key) {
            return bucket[idx].entries[pair_idx].value;
        }
    }
    std::cerr << "No matching key " << *key << std::endl;
    abort();
}

struct Func {
    QString name;
    int     argCount;
    bool    hasResult;
    Func() = default;
    Func(QString _name, int _argc, bool _hasRet)
        : name(_name), argCount(_argc), hasResult(_hasRet){};
};

enum class OPcode
{
    Load,
    Pop,
    Push,
    Call,
    JumpIfFalse,
    Jump,
    NewCall,
    Ass
};

struct Op {
    OPcode opcode;
    Value  arg;
    Func   func;
    Op() = default;
    Op(Value arg1) : opcode(OPcode::Load), arg(arg1){};
    Op(Func _func) : opcode(OPcode::Call), func(_func){};
    Op(OPcode _opcode) : opcode(_opcode){};
};

std::ostream& operator<<(std::ostream& os, OPcode op) {
    switch (op) {
        case OPcode::Load: return os << "Load";
        case OPcode::Pop: return os << "Pop";
        case OPcode::Push: return os << "Push";
        case OPcode::Call: return os << "Call";
        case OPcode::JumpIfFalse: return os << "JumpIfFalse";
        case OPcode::Jump: return os << "Jump";
        case OPcode::NewCall: return os << "NewCall";
        case OPcode::Ass: return os << "Ass";
    }
}

std::ostream& operator<<(std::ostream& os, Op op) {
    os << op.opcode << " ";
    switch (op.opcode) {
        case OPcode::Load: {
            os << op.arg;
            break;
        }
        case OPcode::Call: {
            os << "[ " << op.func.name.toStdString() << " ]"
               << "(" << op.func.argCount << ")";
            break;
        }
        case OPcode::Ass: {
            break;
        }
        default: break;
    }

    return os;
}

std::vector<Op> ast_to_code(Ast* ast) {
    std::vector<Op>                   result;
    std::unordered_map<QString, bool> hasResultMap(
        {{"print", false},
         {"add", false},
         {"get", true},
         {"set", false},
         {">", true},
         {"+", true},
         {"-", true},
         {"<", true},
         {"*", true},
         {"/", true},
         {"==", true},
         {"receive", true}});

    switch (ast->kind) {
        case Ast_Kind::NUMBER: {
            result.push_back(Op(Value(ast->str.toInt())));
            break;
        }
        case Ast_Kind::VAR: {
            result.push_back(Op(Value(ValueKind::Var, ast->str)));
            break;
        }

        case Ast_Kind::STR: {
            result.push_back(Op(Value(ValueKind::String, ast->str)));
            break;
        }

        case Ast_Kind::function_call:
        case Ast_Kind::OP: {
            for (Ast* node : ast->subnodes) {
                auto subprogram = ast_to_code(node);
                result.insert(result.end(), subprogram.begin(), subprogram.end());
            }
            result.push_back(Op(Func(ast->str, ast->subnodes.size(), hasResultMap[ast->str])));
            break;
        }

        case Ast_Kind::assign_statement: {
            for (Ast* node : ast->subnodes) {
                auto subprogram = ast_to_code(node);
                result.insert(
                    result.end(), subprogram.begin(), subprogram.end());
            }
            result.push_back(Op(OPcode::Ass));
            break;
        }

        case Ast_Kind::value:
        case Ast_Kind::expr:
        case Ast_Kind::lang:
        case Ast_Kind::statement:
        case Ast_Kind::if_branch:
        case Ast_Kind::else_branch:
        case Ast_Kind::if_condition:
        case Ast_Kind::else_head:
        case Ast_Kind::if_body:
        case Ast_Kind::else_body:
        case Ast_Kind::while_condition:
        case Ast_Kind::while_body: {
            for (Ast* node : ast->subnodes) {
                auto subprogram = ast_to_code(node);
                result.insert(
                    result.end(), subprogram.begin(), subprogram.end());
            }
            break;
        }
        case Ast_Kind::if_check: {
            auto condition = ast_to_code(ast->subnodes[0]->subnodes[0]);
            auto body = ast_to_code(ast->subnodes[0]->subnodes[1]);
            result.insert(result.end(), condition.begin(), condition.end());
            result.push_back(Op(Value(body.size() + 3)));
            result.push_back(Op(OPcode::JumpIfFalse));
            result.insert(result.end(), body.begin(), body.end());
            if (ast->subnodes.size() == 2) {
                auto else_body = ast_to_code(ast->subnodes[1]->subnodes[1]);
                result.push_back(Op(Value(else_body.size() + 1)));
                result.push_back(Op(OPcode::Jump));
                result.insert(result.end(), else_body.begin(), else_body.end());
            }
            break;
        }
        case Ast_Kind::while_statement: {
            auto condition = ast_to_code(ast->subnodes[0]);
            auto body = ast_to_code(ast->subnodes[1]);
            result.insert(result.end(), condition.begin(), condition.end());
            result.push_back(Op(Value(body.size() + 3)));
            result.push_back(Op(OPcode::JumpIfFalse));
            result.insert(result.end(), body.begin(), body.end());
            result.push_back(Op(Value(-(condition.size() + body.size() + 3))));
            result.push_back(Op(OPcode::Jump));
        }
        case Ast_Kind::new_expr: {
            if (ast->str == "LinkedList") {
                result.push_back(Op(Value(new Node())));
            } else if (ast->str == "HashTable") {
                result.push_back(Op(Value(new HashMapTable())));
            }
        }
    }
    return result;
}

Value evalFunc(QString name, std::vector<Value> args) {
    if (name == "+") {
        return Value(args[0].getIntVal() + args[1].getIntVal());
    } else if (name == "*") {
        return Value(args[0].getIntVal() * args[1].getIntVal());
    } else if (name == "/") {
        return Value(args[0].getIntVal() / args[1].getIntVal());
    } else if (name == "==") {
        return Value(args[0].getIntVal() == args[1].getIntVal());
    } else if (name == "-") {
        return Value(args[0].getIntVal() - args[1].getIntVal());
    } else if (name == "print") {
        std::cout << "print: ";
        for (const auto& arg : args) {
            std::cout << arg << " ";
        }
        std::cout << std::endl;
    } else if (name == "<") {
        return Value(args[0].getIntVal() < args[1].getIntVal());
    } else if (name == ">") {
        return Value(args[0].getIntVal() > args[1].getIntVal());
    } else if (name == "add") {
        args[0].getListVal()->insert(args[1].newCopy());
    } else if (name == "get") {
        return *args[0].getListVal()->get(args[1].getIntVal());
    } else if (name == "set") {
        args[0].getTableVal()->insert(
            args[1].newCopy(), args[2].newCopy());
    } else if (name == "receive") {
        auto res = args[0].getTableVal()->get_table(args[1].newCopy());
        return *res;
    }
};

std::pair<QMap<QString, Value>, std::vector<Value>> eval_code(
    std::vector<Op> program,
    bool printRun) {
    int programCounter = 0;
    QMap<QString, Value> var_table;
    std::vector<Value>   stack;

    while (programCounter < program.size()) {
        auto cmd = program[programCounter];
        if (printRun) {

            std::cout << std::left << "@" << std::setw(4) << programCounter
                      << cmd << std::endl;
        }
        switch (cmd.opcode) {
            case OPcode::Load: {
                stack.push_back(cmd.arg);
                ++programCounter;
                break;
            }
            case OPcode::Call: {
                std::vector<Value> args;
                for (int i = 0; i < cmd.func.argCount; ++i) {
                    auto value = stack.back();
                    if (value.getKind() == ValueKind::Var) {
                        value = var_table[value.getVarVal()];
                    }
                    args.push_back(value);
                    stack.pop_back();
                }

                std::reverse(args.begin(), args.end());

                auto res = evalFunc(cmd.func.name, args);
                if (cmd.func.hasResult) {
                    stack.push_back(res);
                } else {
                }
                ++programCounter;
                break;
            }
            case OPcode::Ass: {
                auto value = stack.back();
                stack.pop_back();
                var_table[stack.back().getVarVal()] = value;
                stack.pop_back();
                ++programCounter;
                break;
            }
            case OPcode::JumpIfFalse: {
                auto jump_address = stack.back();
                stack.pop_back();
                auto condition = stack.back();
                stack.pop_back();
                if (condition.getBoolVal() == true) {
                    ++programCounter;
                } else {
                    programCounter = programCounter + jump_address.getIntVal();
                }
                break;
            }
            case OPcode::Jump: {
                auto jump_address = stack.back();
                stack.pop_back();
                programCounter = programCounter + jump_address.getIntVal();
                break;
            }
        }
    }
    return {var_table, stack};
}


#define DO_DEBUG 1
#define DEBUG_OPC 1
#define DEBUG_LEX 1

int main(int argc, char** argv) {

    QString code;

#if DO_DEBUG
    argc    = 3;
    argv[1] = "file";
    argv[2] = "code.txt";


    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }
#endif

    QCoreApplication   app(argc, argv);
    QCommandLineParser optParser;
    optParser.addHelpOption();
    optParser.addPositionalArgument("mode", "Input method");
    optParser.addPositionalArgument("input", "input data");

    QCommandLineOption astOpt("a", "Print ast");
    optParser.addOption(astOpt);

    QCommandLineOption lexOpt("l", "Print lexer output");
    optParser.addOption(lexOpt);

    QCommandLineOption progOpt("p", "Print compiled bytecode");
    optParser.addOption(progOpt);

    QCommandLineOption runOpt("r", "Print program execution");
    optParser.addOption(runOpt);

    optParser.process(app);

    const QStringList args = optParser.positionalArguments();

    if (args.length() < 2) {
        std::cerr << "Missing input source " << args.length() << "\n";
        optParser.showHelp();
        abort();
    }


    if (args[0] == "file") {
#if DO_DEBUG
        QDir dir      = QFileInfo(__FILE__).absoluteDir();
        auto filePath = dir.absoluteFilePath(args[1]);
        std::cout << filePath.toStdString() << std::endl;
        QFile file(filePath);
#else
        QFile file(args[1]);
#endif
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QTextStream stream(&file);
        code = stream.readAll();
    } else if (args[0] == "script") {
        code = args[1];
    } else {
        std::cerr
            << "Expected either `script` of `file` option, but found "
            << args[0] << "\n";
        optParser.showHelp();
        abort();
    }


    auto parser = new Parser(
        lexer(code, optParser.isSet(lexOpt) || DEBUG_LEX));
    auto ast = parser->parse();
    if (optParser.isSet(astOpt)) {
        ast->print(0);
    }

    std::cout << std::endl;
    std::vector<Op> program = ast_to_code(ast);

    if (optParser.isSet(progOpt) || DEBUG_OPC) {
        for (size_t i = 0; i < program.size(); ++i) {
            std::cout << std::left << std::setw(6) << i << " "
                      << program[i] << std::endl;
        }
    }


    auto result = eval_code(program, optParser.isSet(runOpt));
    std::cout << "--- variable table ---\n";
    for (const auto& key : result.first.keys()) {
        std::cout << key.toStdString() << " = " << result.first[key]
                  << "\n";
    }

    std::cout << "--- stack values---\n";
    for (const auto& val : result.second) {
        std::cout << val << "\n";
    }
}
