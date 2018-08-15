%
% LaTeX2e Style for MAS R&D and master thesis reports
% Author: Argentina Ortega Sainz, Hochschule Bonn-Rhein-Sieg, Germany
% Please feel free to send issues, suggestions or pull requests to:
% https://github.com/mas-group/project-report
% Based on the template created by Ronni Hartanto in 2003
%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{mas_report}[2018/01/25 The MAS report class]

\RequirePackage{xcolor}
\RequirePackage{ifthen}
\definecolor{slcolor}{HTML}{882B21}

\newcommand*{\project}[1]{\gdef\@project{#1}%
}
\newcommand*{\@project}{\texttt{project} not set. Please select Master's Thesis or R\&D Project}

% Options
\DeclareOption{onecolumn}{\OptionNotUsed}
\DeclareOption{thesis}{\project{Master's Thesis}}
\DeclareOption{rnd}{\project{R\&D Project}}

% Report options
\DeclareOption{twosides}{
  \PassOptionsToClass{\CurrentOption}{book}
}
\DeclareOption{oneside}{
    \PassOptionsToClass{\CurrentOption}{book}
}
% Execute the default option
\ExecuteOptions{rnd}
\ExecuteOptions{twosides}
% Process other options
\ProcessOptions\relax

% Load class options
\LoadClass[12pt,a4paper]{book}

% \renewcommand{\sfdefault}{phv}
% \renewcommand{\familydefault}{\sfdefault}

%-------------------------------------------------------------------------------
%   VARIOUS REQUIRED PACKAGES AND CONFIGURATIONS
%-------------------------------------------------------------------------------

\usepackage[numbers]{natbib}\citeindextrue % Round brackets around citations, change to square for square brackets
\usepackage{graphicx} % Required to include images
\usepackage{amsmath,amssymb,theorem} % Math packages
\usepackage{listings} % Required for including snippets of code
\usepackage{booktabs} % Required for better horizontal rules in tables
\usepackage{microtype} % Slightly tweak font spacing for aesthetics
\usepackage[titletoc]{appendix}
\usepackage{setspace}
\usepackage{subfiles}
\usepackage{geometry}
\usepackage{hyperref}
\usepackage[bottom,perpage]{footmisc}
% \usepackage{showframe}
\usepackage{tabularx}
\usepackage{array}
\newcolumntype{L}[1]{>{\raggedright\let\newline\\\arraybackslash\hspace{0pt}}p{#1}}
\newcolumntype{C}[1]{>{\centering\let\newline\\\arraybackslash\hspace{0pt}}p{#1}}
\newcolumntype{R}[1]{>{\raggedleft\let\newline\\\arraybackslash\hspace{0pt}}p{#1}}
\usepackage{subcaption}

\usepackage{todonotes}

\onehalfspacing

\graphicspath{{../images/}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definitions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\renewcommand\bibname{References} %Change Bibliography to References

\newcommand{\thirdpartylogo}[1]{\def\@thirdpartylogo{#1}}
\thirdpartylogo{}

\newcommand*{\supervisors}[1]{\gdef\@supervisors{#1}%
}
\newcommand*{\@supervisors}{\texttt{\string\supervisors} currently
not set. Please fix this.}

% Colors
\definecolor{hbrs}{RGB}{1,106,186}

\definecolor{keywords}{RGB}{0,100,0}
\definecolor{comments}{RGB}{0,100,50}
\definecolor{stringred}{RGB}{160,0,0}
\definecolor{codeblue}{RGB}{0,0,150}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title page
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\renewcommand*{\maketitle}{%
\begin{titlepage}
    \newgeometry{left=3cm, bottom=3cm}

    \begin{minipage}{0.475\textwidth}
        \centering
       \includegraphics[height=1.3cm]{Logo-HBRS-cyan.png}
    \end{minipage}
    \begin{minipage}{0.475\textwidth}
        \centering
       \includegraphics[height=1.3cm]{b-it.pdf}
    \end{minipage}
    \centering
    \vfill
    \ifx\@thirdpartylogo\@empty
    \relax
    \else
    \begin{minipage}{0.4\textwidth}
        \centering
       \includegraphics[height=1.3cm]{\@thirdpartylogo}
    \end{minipage}

    \fi%

\vfill
{\Large\@project\unskip\strut\par}
\vspace{1.5cm}
{\huge\bfseries\@title\unskip\strut\par}
\vspace{2cm}
{\Large\itshape\@author\unskip\strut\par}
\vfill

{\normalsize Submitted to Hochschule Bonn-Rhein-Sieg,\\
Department of Computer Science\\
in partial fullfilment of the requirements for the degree\\
of Master of Science in Autonomous Systems}
\vfill

Supervised by\par
\vskip 8pt
{\large
\@supervisors\unskip\strut\par
}
\vfill

{\large \@date\par}
\end{titlepage}
\restoregeometry
}


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Declaration of authorship
%%%%%%%%%%%%%%%%%%%%%%%%%%%
\def\statementpage{%
    \thispagestyle{empty}
    \hbox{}
    \vfill
    \noindent
    I, the undersigned below, declare that this work has not previously been
    submitted to this or any other university and that it is, unless otherwise stated,
    entirely my own work.
    \vskip 60pt

    \hrule width 4cm
    \hfill \hspace{5cm}
    \hrulefill

    \vskip -3pt

    \leftline{\hspace{1.5cm} Date \hfill \@author \hspace{1.5cm}}

    \vskip 60pt
    \clearpage
}%

%%%%%%%%%%%%%%%%%%%%
% Copyright
%%%%%%%%%%%%%%%%%%%%

\def\copyrightpage{%
    \hbox{ }
    \vfill
    \begin{center}
        \copyright\ Copyright \@copyrightyear \\
        by \\
        \@author \\
        All Rights Reserved \\ [12pt]
    \end{center}
    \clearpage
}%


%%%%%%%%%%%%%%%%%%%%%
% Abstract
%%%%%%%%%%%%%%%%%%%%%

\newenvironment{abstract}{%
\chapter*{Abstract}
}


%%%%%%%%%%%%%%%%%%%%%%%%%%
% Acknowledgements
%%%%%%%%%%%%%%%%%%%%%%%%%%
\newenvironment{acknowledgements}{%
\chapter*{Acknowledgements}
% \cleardoublepage\thispagestyle{empty}\null\vfill\begin{center}%
    % \bfseries Acknowledgements\end{center}

}
%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PAGE LAYOUT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIDE MARGINS:
% \oddsidemargin 28pt      %   Left margin on odd-numbered pages.
% \evensidemargin 0pt      %   Left margin on even-numbered pages.
% \marginparwidth 40pt     %   Width of marginal notes.
%
% \marginparsep 12pt          % Horizontal space between outer margin and
%                             % marginal note
% \textwidth 15cm             % width of text
%
% % VERTICAL SPACING:
%                          % Top of page:
% \topmargin  10pt          %    distance from top of page to running head
% \headheight 14pt         %    Height of box containing running head.
% \headsep  1cm            %    Space between running head and text.
% \textheight 8.8in        %    space for text
% \footskip 30pt           %    Distance from baseline of box containing foot
%                          %    to baseline of last line of text.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Header and footer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage{titlesec}

\let\footruleskip\undefined
\usepackage{fancyhdr}% http://ctan.org/pkg/fancyhdr

%% Page style for title pages.
\fancypagestyle{plain}{%
    \fancyhf{}
    \renewcommand*\headrulewidth{0pt}
    \renewcommand*\footrulewidth{0pt}
    \fancyfoot[C]{\thepage}
    % \fancyfoot[C]{}
}

%% Fancy style for the main matter.
\fancypagestyle{mainmatter}{%
    \fancyhf{}
    \renewcommand*\headrulewidth{0.5pt}
    %% Page numbers on the top left and top right.
    \fancyfoot[C]{\thepage}
    %% Chapter name on the left (even) page.
    \fancyhead[RE]{\nouppercase{\rightmark}}
    %% Section name on the right (odd) page.
    \fancyhead[LO]{\nouppercase{\leftmark}}
}


%% Print the current chapter and section at the top of the page in cyan.
\renewcommand*\chaptermark[1]{\markboth{\textcolor{hbrs} {\thechapter.\ } #1}{}}
\renewcommand*\sectionmark[1]{\markright{\textcolor{hbrs}{\thesection.\ } #1}}

%% Chapter titles have the same layout as parts.
\titleformat{\chapter}
    [display] % sectioning paragraph shape
    {\flushright}% format to be applied to the title, label, and text
    {\color{gray}\fontsize{68pt}{68pt}\selectfont\bfseries\thechapter}% sectioning label
    {0pt}% horizontal separation between label and title body and it must be a length and not be empty.
    {\Huge\color{hbrs}}% code preceding the title body
    [\vspace{1cm}] % code following the title body (optional)

\titlespacing{\chapter}{0pt}{0pt}{\baselineskip}

\titleformat{\section}
    [block]
    {\flushleft}%format
    {}%label
    {0pt}%sep
    {\bfseries\large\thesection~}%before code
    % [\vspace{0.5cm}]

\titlespacing{\section}{0pt}{0pt}{5pt}

\titleformat{\subsection}[display]
    {\flushleft}%format
    {}%label
    {0pt}%sep
    {\bfseries\large\thesubsection~}%before code
    % [\vspace{0.5cm}]

\titlespacing{\subsection}{0pt}{0pt}{2pt}

%\renewcommand{\cfttoctitlefont}{\color{hbrs}}
%\renewcommand{\cftlottitlefont}{\flushright\color{hbrs}}
%\renewcommand{\cftloftitlefont}{\flushright\color{hbrs}}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional macros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% #1 - image path, #2 - caption, #3 - label
\newcommand{\picHere}[3] { %
    \begin{figure}[h!] %
        \begin{center} %
            \resizebox{!}{!}{\includegraphics{#1}} %
            \caption{{\small#2}} %
            \label{#3} %
        \end{center} %
    \end{figure} %
}

% #1 - image path, #2 - caption, #3 - label, #4 - figure width
\newcommand{\picHereWidth}[4] { %
    \begin{figure}[h!] %
        \begin{center} %
            \resizebox{!}{!}{\includegraphics[width=#4]{#1}} %
            \caption{{\small #2}} %
            \label{#3} %
        \end{center} %
    \end{figure} %
}

% #1 - image path, #2 - caption, #3 - label
\newcommand{\picHereRes}[3] { %
    \begin{figure}[h!] %
        \begin{center} %
            \resizebox{!}{!}{\includegraphics[totalheight=0.8\textheight]{#1}} %
            \caption{#2} %
            \label{#3} %
        \end{center} % 
    \end{figure} %
}

% #1 - link, #2 - display text, #3 - display link
\newcommand{\footnoteHrefSpecial}[3]{\href{#1}{#2 \footnote{\href{#1}{#3}}}}

% #1 - link, #2 - display text (display link = link)
\newcommand{\footnoteHref}[2]{\footnoteHrefSpecial{#1}{#2}{#1}}

\endinput