# Shortcut report

Training file `demos\jeff\data\split\train.jsonl`: 7,750 rows, 31 families. Test file `demos\jeff\data\split\test.jsonl`: 1,750 rows, 7 families.


## Flags

- A model that sees only all surface features (adding state size, option count and instructions) predicts the label with 33.2% balanced accuracy on test, against 7.7% by chance (z = 34.4).
- A model that sees only the form of the main text (length, punctuation, case, digits) tells 'egress' from the other labels with 62.4% balanced accuracy on test, against 50.0% by chance (z = 4.8).
- A model that sees only the form of the main text (length, punctuation, case, digits) tells 'false' from the other labels with 60.3% balanced accuracy on test, against 50.0% by chance (z = 4.8).
- A model that sees only the form of the main text (length, punctuation, case, digits) tells 'transit' from the other labels with 72.1% balanced accuracy on test, against 50.0% by chance (z = 8.5).
- 70 phrases are mostly found in one label (see section 4). Words that carry the meaning are expected; style words of one generation round or one source are shortcuts.
- Rows of kind 'em' are 0.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'em' are 33.3% 'emcon' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'em' are 33.3% 'illum' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'em' are 33.3% 'tws' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'engage_auth' are 0.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'engage_auth' are 50.0% 'false' against 8.4% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'engage_auth' are 50.0% 'true' against 8.4% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'intent' are 0.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'intent' are 33.3% 'egress' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'intent' are 33.3% 'ingress' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'intent' are 33.3% 'transit' against 5.6% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'phase' are 0.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'phase' are 26.2% 'alert' against 4.4% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'phase' are 23.8% 'engage' against 4.0% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'phase' are 23.8% 'reset' against 4.0% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'phase' are 26.2% 'watch' against 4.4% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'profile' are 100.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Rows of kind 'terminal' are 100.0% '<listed option>' against 32.8% overall: if this kind is recognisable by its style, the style predicts the label.
- Surface features tell the row kinds apart (53.3% balanced accuracy against 16.7%, z = 41.1): each kind has its own style.

## 1. Label balance

| label | train | test | train rows |
|---|---|---|---|
| <listed option> | 32.8% | 32.8% | 2542 |
| alert | 4.4% | 4.4% | 341 |
| egress | 5.6% | 5.6% | 434 |
| emcon | 5.6% | 5.6% | 434 |
| engage | 4.0% | 4.0% | 310 |
| false | 8.4% | 8.4% | 651 |
| illum | 5.6% | 5.6% | 434 |
| ingress | 5.6% | 5.6% | 434 |
| reset | 4.0% | 4.0% | 310 |
| transit | 5.6% | 5.6% | 434 |
| true | 8.4% | 8.4% | 651 |
| tws | 5.6% | 5.6% | 434 |
| watch | 4.4% | 4.4% | 341 |

Always answering the most common training label ('<listed option>') would score 32.8% on test; uniform chance is 7.7%.
`<listed option>` stands for any per-row listed option: a key such as o1 ... o12 that names a different item from row to row (used by fewer than 20% of training choice rows, or with no one description in half of them).

## 2. The correct option: position and length

| split | rows compared | correct is longest | correct is shortest | correct is first | correct is last | mean relative position (0 first, 1 last; 0.50 expected) |
|---|---|---|---|---|---|---|
| train | 6448 | 32.2% (chance 32.5%) | 34.2% (chance 34.1%) | 32.4% (chance 31.7%) | 31.3% (chance 31.7%) | 0.49 |
| test | 1456 | 31.8% (chance 32.3%) | 34.6% (chance 34.1%) | 31.7% (chance 31.7%) | 31.0% (chance 31.7%) | 0.50 |

Compared among the listed options when the answer is one of them, among all options when every option is fixed; rows whose answer is a fixed option are left out (their share is the label balance above). Chance counts ties: if two options share the greatest length, either counts as longest.

## 3. Models that see only surface features

Each model is trained on the training file and scored on the test file. None of them reads a word of meaning. If one beats chance clearly, the data contains a pattern Jeff could learn instead of the task.

**Main text length by label (characters)**

| label | split | rows | 10th percentile | median | 90th percentile |
|---|---|---|---|---|---|
| <listed option> | train | 2542 | 190 | 241 | 298 |
| <listed option> | test | 574 | 190 | 234 | 293 |
| alert | train | 341 | 191 | 246 | 305 |
| alert | test | 77 | 198 | 246 | 292 |
| egress | train | 434 | 189 | 237 | 295 |
| egress | test | 98 | 189 | 239 | 297 |
| emcon | train | 434 | 191 | 248 | 299 |
| emcon | test | 98 | 185 | 244 | 309 |
| engage | train | 310 | 198 | 247 | 299 |
| engage | test | 70 | 190 | 250 | 310 |
| false | train | 651 | 189 | 238 | 298 |
| false | test | 147 | 185 | 242 | 305 |
| illum | train | 434 | 193 | 246 | 301 |
| illum | test | 98 | 189 | 236 | 303 |
| ingress | train | 434 | 197 | 245 | 299 |
| ingress | test | 98 | 191 | 247 | 294 |
| reset | train | 310 | 185 | 239 | 293 |
| reset | test | 70 | 187 | 240 | 304 |
| transit | train | 434 | 189 | 239 | 301 |
| transit | test | 98 | 198 | 243 | 299 |
| true | train | 651 | 190 | 245 | 300 |
| true | test | 147 | 200 | 254 | 301 |
| tws | train | 434 | 190 | 239 | 304 |
| tws | test | 98 | 196 | 238 | 299 |
| watch | train | 341 | 195 | 246 | 293 |
| watch | test | 77 | 195 | 230 | 299 |

| model (predicts the label) | accuracy | balanced accuracy | chance |  |
|---|---|---|---|---|
| the length of the main text (characters and words) | 5.7% | 8.9% | 7.7% |  |
| the form of the main text (length, punctuation, case, digits) | 10.2% | 13.4% | 7.7% |  |
| all surface features (adding state size, option count and instructions) | 28.1% | 33.2% | 7.7% | **flag** |

**Strongest single features** (balanced accuracy on test from one feature alone): options (log) 23.1%; digit share 10.8%; most common instructions 10.0%; count of , 8.2%; count of - 8.2%; capital letter share 8.2%

**One label against all the others** (balanced accuracy on test; chance 50%):

| label | the length of the main text (characters and words) | the form of the main text (length, punctuation, case, digits) |
|---|---|---|
| <listed option> | 52.7% | 53.1% |
| alert | 50.9% | 53.4% |
| egress | 52.5% | 62.4% **flag** |
| emcon | 51.5% | 51.8% |
| engage | 54.0% | 58.1% |
| false | 48.7% | 60.3% **flag** |
| illum | 51.7% | 54.0% |
| ingress | 57.4% | 52.9% |
| reset | 53.8% | 58.2% |
| transit | 52.1% | 72.1% **flag** |
| true | 54.2% | 58.6% |
| tws | 48.8% | 48.0% |
| watch | 46.4% | 58.2% |

**Option picker without meaning**: scores each option from its position, length and shape (commas, brackets, digits, capitals), never its words or the input, and picks the top option per row. Test accuracy 32.3% against chance 31.7% over 1456 rows.

## 4. Words and punctuation by label

Share of each label's training rows whose main text has the feature. One kind of request missing filler words, or questions always having articles, lets a model tell labels apart by style.

| feature (share of training rows) | <listed option> | alert | egress | emcon | engage | false | illum | ingress | reset | transit | true | tws | watch |  |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| filler word (um, uh, er, hmm) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| starts with a question word | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has a question mark | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has an article (a, an, the) | 91.2% | 95.6% | 91.5% | 90.1% | 92.3% | 92.9% | 91.7% | 92.6% | 89.7% | 94.2% | 90.8% | 91.7% | 91.2% |  |
| ends with . | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% |  |
| ends with ! | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| no end punctuation | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| starts lowercase | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| all lowercase | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has a digit | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% |  |
| has a line break | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has quotes | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has markup (HTML or Markdown) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| non-ASCII character | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| emoji | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| ALL-CAPS word (4+ letters) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |

**Phrases mostly found in one label** (1 to 3 words, in more than 2% of the label's rows, at least 70% of the rows with it in that label, at least 10 rows): transit: `range steady` (70.7% of the label, 87.0% of its 353 rows); transit: `range steady altitude` (35.3% of the label, 87.9% of its 174 rows); transit: `a parallel course` (34.3% of the label, 86.6% of its 172 rows); transit: `course range steady` (34.3% of the label, 86.6% of its 172 rows); transit: `on a parallel` (34.3% of the label, 86.6% of its 172 rows); transit: `parallel course range` (34.3% of the label, 86.6% of its 172 rows); transit: `holding constant range` (29.3% of the label, 87.0% of its 146 rows); false: `friendly` (25.0% of the label, 100.0% of its 163 rows); transit: `km on a` (24.2% of the label, 90.5% of its 116 rows); transit: `km range steady` (24.2% of the label, 86.8% of its 121 rows); transit: `km holding constant` (18.2% of the label, 86.8% of its 91 rows); transit: `constant range altitude` (12.7% of the label, 82.1% of its 67 rows); <listed option>: `29 5 km` (10.1% of the label, 96.3% of its 267 rows); transit: `range steady flying` (9.7% of the label, 89.4% of its 47 rows); transit: `steady flying at` (9.7% of the label, 89.4% of its 47 rows); transit: `range steady height` (8.8% of the label, 90.5% of its 42 rows); false: `answered friendly contact` (8.6% of the label, 100.0% of its 56 rows); false: `challenge answered friendly` (8.6% of the label, 100.0% of its 56 rows); false: `iff challenge answered` (8.6% of the label, 100.0% of its 56 rows); false: `iff responded presumed` (8.4% of the label, 100.0% of its 55 rows); false: `responded presumed friendly` (8.4% of the label, 100.0% of its 55 rows); false: `4 correct friendly` (8.0% of the label, 100.0% of its 52 rows); false: `iff mode 4` (8.0% of the label, 100.0% of its 52 rows); false: `mode 4 correct` (8.0% of the label, 100.0% of its 52 rows); false: `friendly the` (6.9% of the label, 100.0% of its 45 rows); transit: `constant range height` (6.2% of the label, 100.0% of its 27 rows); transit: `range steady at` (6.0% of the label, 76.5% of its 34 rows); false: `friendly track` (5.8% of the label, 100.0% of its 38 rows); false: `s iff responded` (5.7% of the label, 100.0% of its 37 rows); false: `friendly the track` (5.1% of the label, 100.0% of its 33 rows)

## 5. Near duplicates and split separation

- Families in both train and test: 0.

- Identical training prompts (state, options and instructions) with different answers: 0.

- Test rows whose main text appears verbatim in train (normalised): 0 of 1750. Short generic texts can be legitimate on different screens or companies; jeff-kit leak-check decides.

- Test rows with a near duplicate in train (main-text similarity of at least 0.8): 0 (0.0%).

- Near-duplicate training pairs with different labels: 0 (can be right when the rest of the state or the options differ).

## 6. Kinds of row (`source.kind`)

**Label mix per kind (train)**

| kind | rows | <listed option> | alert | egress | emcon | engage | false | illum | ingress | reset | transit | true | tws | watch |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| em | 1302 | 0.0% | 0.0% | 0.0% | 33.3% | 0.0% | 0.0% | 33.3% | 0.0% | 0.0% | 0.0% | 0.0% | 33.3% | 0.0% |
| engage_auth | 1302 | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 50.0% | 0.0% | 0.0% | 0.0% | 0.0% | 50.0% | 0.0% | 0.0% |
| intent | 1302 | 0.0% | 0.0% | 33.3% | 0.0% | 0.0% | 0.0% | 0.0% | 33.3% | 0.0% | 33.3% | 0.0% | 0.0% | 0.0% |
| phase | 1302 | 0.0% | 26.2% | 0.0% | 0.0% | 23.8% | 0.0% | 0.0% | 0.0% | 23.8% | 0.0% | 0.0% | 0.0% | 26.2% |
| profile | 1271 | 100.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |
| terminal | 1271 | 100.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |
| all | 7750 | 32.8% | 4.4% | 5.6% | 5.6% | 4.0% | 8.4% | 5.6% | 5.6% | 4.0% | 5.6% | 8.4% | 5.6% | 4.4% |

A surface-feature model tells the kinds apart on test with balanced accuracy 53.3% against 16.7% by chance **flag**.

**Words and punctuation by kind (train)**

| feature (share of training rows) | em | engage_auth | intent | phase | profile | terminal |  |
|---|---|---|---|---|---|---|---|
| filler word (um, uh, er, hmm) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| starts with a question word | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has a question mark | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has an article (a, an, the) | 91.2% | 91.9% | 92.8% | 92.2% | 91.0% | 91.4% |  |
| ends with . | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% |  |
| ends with ! | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| no end punctuation | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| starts lowercase | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| all lowercase | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has a digit | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% |  |
| has a line break | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has quotes | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| has markup (HTML or Markdown) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| non-ASCII character | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| emoji | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |
| ALL-CAPS word (4+ letters) | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |  |

## 7. Leftovers and personal data

| pattern (whole state and options, train and test) | rows | examples |
|---|---|---|
| unfilled template slot ({{...}}, [NAME], <NAME>) | 0 |  |
| lorem ipsum | 0 |  |
| TODO / TBD / FIXME | 0 |  |
| 'As an AI' or a refusal | 0 |  |
| chat preamble ('Here is...', 'Sure!') | 0 |  |
| label words ('Example 3:', 'Output:') | 0 |  |
| model control tokens (<think>, <\|im_start\|>) | 0 |  |
| garbled encoding | 0 |  |
| email address (personal data?) | 0 |  |
| phone number (personal data?) | 0 |  |
| URL | 0 |  |
| HTML tag | 0 |  |

## How the flags are decided

- **Beats chance clearly**: at least 10% (percentage points) above chance and at least 3 standard errors
  above it, where the standard error is that of guessing at chance on the same rows (for balanced accuracy, with
  each label's number of test rows). Small files and rare labels need a bigger lead.
- **Label balance**: a label's share differs between train and test by more than 5% (percentage points).
- **One label against the rest**: the same "beats chance clearly" rule against 50%, for each label with at least
  5 training rows (run when there are 3 or more labels).
- **Option picker, option position and option length**: the same rule, against the chance of guessing among each row's
  options.
- **Words and punctuation**: shares differ by at least 15% (percentage points), the highest is at least
  1.8 times the lowest, and they are at least 3 standard errors apart (two-proportion test); labels or
  kinds with fewer than 5 training rows are left out.
- **Phrases**: in more than 2% of one label's rows, at least 70% of the rows
  containing it in that label (and at least twice the label's share), and in at least 10 rows.
- **Kinds of row**: a label's share inside a kind (of at least 5 rows) is more than 10%
  (percentage points) from its share overall; a surface model telling the kinds apart uses the "beats chance" rule.
- **Near duplicates**: more than 2% of test rows have a training row with main-text similarity of at
  least 0.8 (Jaccard similarity of word 3-grams, or of character 5-grams for texts under 5 words).
- **Leftovers**: any row matching a flagged pattern in section 7.

A flag asks for a person to look. Some are expected: a "question" label should end with "?" more often. A flag is a
problem when the feature is not what the label means, or when it would not hold for real inputs.
