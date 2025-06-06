"use client";
// components/CodeBlock.tsx
import React, { useState } from "react";
import SyntaxHighlighter from "react-syntax-highlighter";
import { ExternalLink, ChevronDown, ChevronRight } from "lucide-react";

interface CodeBlockProps {
  code: string;
  language?: string;
  title?: string;
  githubUrl?: string;
}

const CodeBlock: React.FC<CodeBlockProps> = ({
  code,
  language = "python",
  title,
  githubUrl,
}) => {
  const [isVisible, setIsVisible] = useState(false);

  return (
    <div className="my-6 border rounded-lg overflow-hidden bg-[#1e1e1e] hover:cursor-pointer" onClick={() => setIsVisible(!isVisible)}>
      <div className="flex items-center justify-between px-4 py-2 bg-gray-900 text-gray-100 text-sm font-mono">
        <div
          className="flex items-center gap-1 text-left focus:outline-none"
        >
          {isVisible ? (
            <ChevronDown className="w-4 h-4" />
          ) : (
            <ChevronRight className="w-4 h-4" />
          )}
          {title || "Code Snippet"}
        </div>
        {githubUrl && (
          <a
            href={githubUrl}
            target="_blank"
            rel="noopener noreferrer"
            className="text-blue-400 hover:underline flex items-center gap-1"
          >
            <ExternalLink className="w-4 h-4" />
            View on GitHub
          </a>
        )}
      </div>
      {isVisible && (
        <SyntaxHighlighter language={language} className="p-4">
          {code.trim()}
        </SyntaxHighlighter>
      )}
    </div>
  );
};

export default CodeBlock;
