# Search and Source-Verification Log

Verification date: **2026-10-10 (America/New_York)**

## Purpose and scope

The starting point was user-provided ChatGPT text listing nine papers. Each was checked against original sources/publication metadata to create an initial reading guide for a two-person project.

Inclusion criteria: existence confirmed through an original source, and relevance to shared-object manipulation, intent recognition, sensor fusion, payload estimation, or tactile control. Papers directly addressing reorientation received priority; related tasks were marked as method references. The scope was these nine English-language seed titles from 2018–2026, not a comprehensive search across that period.

Sources were found for all nine papers. Versions were aligned by title/DOI, with no duplicate independent seed entries identified. Workshops are not papers and were not added to the reading list. Complete search-result counts were not retained, so no PRISMA flow or complete recall estimate is reported.

## Search method

Searches used full titles with first author/year. Main sources included author pages, Honda Research Institute, the TU Delft repository, Frontiers, IEEE indexes, Springer, Wiley, Nature, and arXiv. P08 used the original paper's abstract indexed by PubMed.

Query families used:

- `"Human-Robot Cooperative Object Manipulation with Contact Changes" Gienger`
- `"Human-Robot Planar Co-Manipulation of Extended Objects" Mielke`
- `"Understanding Human Co-Manipulation via Motion and Haptic Information"`
- `"Vision- and tactile-based continuous multimodal" "10.1109"`
- `"Robot to Human Object Handover Using Vision" site:link.springer.com`
- `"Sensorless Human–Robot Interaction" "Mass" "Wrench" Bai`
- `"TactileReflex" IROS 2026 project`
- `"Multistep Intent Estimation Guided Adaptive Passive Control"`
- `"Embodied Sensorimotor Integration for Whole-Arm Tactile Sensing"`

## Inspection scope

| ID | Metadata / existence evidence | Content inspected | Not completed |
| --- | --- | --- | --- |
| P01 | TU Delft metadata + Honda accepted manuscript | Key sections including III, VI, and VII | Team full-text reading / code and control reproduction |
| P02 | Frontiers / PMC | Methods, experiments, 5.3–5.4, conclusion | Team full-text reading / raw-data reanalysis |
| P03 | Frontiers / PMC | Results, Conclusion | Team full-text reading / raw-data reanalysis |
| P04 | IEEE metadata + arXiv | Abstract and version alignment | Full-text methods / experiment verification |
| P05 | Springer chapter metadata + arXiv | Abstract and version alignment | Full-text methods / experiment verification |
| P06 | Wiley | Abstract, 2 Experimental Section, 2.1–2.4 | Full review of assumptions / control stability |
| P07 | arXiv v4 + author project | Abstract / version / project-method description | Full text / formal proceedings DOI |
| P08 | PubMed PMID 41071693 | Original-paper abstract and metadata | IEEE full text; the DOI page could not be opened with the web tool during this check |
| P09 | Nature search index + author laboratory page | Abstract and metadata | Direct Nature open failed; full text / supplement not inspected |

Each [Reading Card](README.md) links to original sources. “Existence confirmed” does not mean all DOIs resolved successfully, all results were reproduced, or every full paper was read. No Semantic Scholar/OpenAlex programmatic verification or comprehensive retraction, COI, or venue-indexing review was performed.

## Corrections and qualifications to the initial pasted material

1. **P01:** The 90° box rotation appears in VII Evaluation and uses a dual-arm system. Interaction decisions in that experiment use forces estimated from joint torques; the hardware also includes F/T and tactile sensors. [Honda manuscript](https://www.honda-ri.de/pubs/pdf/3729.pdf)
2. **P02:** Safety/confusion/predictability preferences cannot be claimed as statistically significant; the paper separately reports a difference in Good Force Amount. [5.3–5.4](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full)
3. **P04:** The preprint is from 2022, online publication from 2023, and formal issue from 2024. Cite the 2024 issue as the primary publication. [IEEE](https://ieeexplore.ieee.org/document/10144527/)
4. **P05:** In addition to the 2022 preprint, a formal Springer chapter was published in 2023. [Springer](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11)
5. **P06:** Online publication was in 2024 and the issue in 2025. Sensorless means no human-side sensor is required; the robot end effector still uses 6D F/T. [Wiley §2](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)
6. **P07:** arXiv v4 and the author page state acceptance to IROS 2026. BibTeX cites the inspected arXiv v4 rather than guessing a proceedings DOI/page range. The v4 author order differs from earlier versions. [arXiv v4](https://arxiv.org/abs/2605.23568v4), [Author page](https://shayfeng.github.io/TactileReflex/)
7. **P08:** The 2026 issue and 2025 in the DOI are not contradictory; evidence is currently limited to the abstract. [PubMed](https://pubmed.ncbi.nlm.nih.gov/41071693/)
8. **P09:** The 2026 Nature Sensors publication can be aligned between the Nature index and laboratory publication page. Do not treat its hardware demonstration as a single-arm co-grasped flipping result. [Nature](https://www.nature.com/articles/s44460-026-00097-1), [Laboratory](https://mirs.hkust.edu.hk/publications/)
9. IROS workshop dates and the UMD course syllabus in the initial text were not necessary for this paper-verification task and were not used as evidence supporting research conclusions.

## Directions for the next literature search

The following are suggested queries for a future search; they have not been executed:

- `("co-manipulation" OR "physical human-robot collaboration") AND ("vision-language model" OR VLM) AND intent`
- `"collaborative object reorientation" single arm admittance`
- `"vision tactile force" intention ablation human robot`
- `"co-manipulation" ("negative results" OR latency OR ambiguity)`

The current list was selected from seed material. Failure to find a direct three-modal example does not establish novelty. Evidence gaps remain in VLMs, single-arm flipping geometry, generalization to real humans, and reproducibility with open code. This is a targeted exploratory review, not a systematic review.

## AI disclosure

Codex assisted with searching, metadata alignment, short summaries, and design inferences. It did not declare any paper read on the team's behalf, run experiments, or claim independent cross-model review. Before citing a result in the formal report, the team should read the relevant source and confirm its locator.
