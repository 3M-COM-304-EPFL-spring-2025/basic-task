import React from 'react';

interface ReportCardProps {
  title: string;
  children: React.ReactNode;
}

const ReportCard: React.FC<ReportCardProps> = ({ title, children }) => {
  return (
    <div className="bg-white shadow-lg rounded-lg p-6 mb-6">
      <h3 className="text-xl font-semibold text-gray-800 mb-4">{title}</h3>
      <div className="text-gray-700">{children}</div>
    </div>
  );
};

export default ReportCard;
