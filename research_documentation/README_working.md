# SoftWorld research documentation

`soft_world_research_working.tex` is a self-contained LaTeX research development
record for the current working configuration. It covers development history,
physics and geometry, constant-curvature reference equations, position/hybrid
control, the 81.818° topology mismatch and correction, virtual strain sensing,
RViz reconstruction, plots, validation evidence, limitations, and reproducibility.

Historical changes, recorded tests, and proposed experiments are distinguished.
It is a technical draft, not a finished thesis or physical-validation claim.
Replace the author placeholder and review it with your research advisor.

## Compile

With a LaTeX distribution containing the standard packages used by the report:

```bash
cd research_documentation
pdflatex -interaction=nonstopmode -halt-on-error soft_world_research_working.tex
pdflatex -interaction=nonstopmode -halt-on-error soft_world_research_working.tex
```

Two passes resolve the table of contents, references, and citations. Alternatively,
upload the `.tex` file to Overleaf as the project's main document. No external
figures, bibliography file, shell escape, or repository-specific absolute paths
are needed to compile it. Overleaf compilation has not been tested here.

No `pdflatex` or `tectonic` executable was available when this document was
created, so a PDF was not generated and LaTeX compilation is not yet verified.

`validation_record_working.txt` retains available numerical excerpts from
earlier tests. It is not a complete raw dataset, a fresh experiment, or a
repeated-trial statistical summary. Preserve complete logs and synchronized
data when running new research experiments.

Keep future report updates in these working documentation files. The report
records an inspected snapshot; later simulator changes must be reflected here
before relying on it as a methods description.
