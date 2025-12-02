import React from 'react';
import Content from '@theme-original/DocItem/Content';
import TranslateButton from '@site/src/components/TranslateButton';

export default function ContentWrapper(props) {
  return (
    <>
      <TranslateButton />
      <Content {...props} />
    </>
  );
}
